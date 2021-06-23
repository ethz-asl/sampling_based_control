#!/usr/bin/env python3

import os
import datetime
import torch
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import csv
from scipy.stats import bernoulli
from actionlib.simple_action_client import SimpleGoalState

from learning import PolicyLearner

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from mppi_ros.msg import Data
import policy_learning.msg

class Dagger:
    """
    Class to handle the generic dataset aggregation loop with virtual methods
    to be completed in the example which are implementation specific.
    """

    def __init__(self, file_path):
        # handle path to initial dataset or (TODO) flag to start from fresh (data collection)
        self.file_path = file_path
        if os.path.isdir(file_path):
            # set device for policy
            device = 'cuda' if torch.cuda.is_available() else 'cpu'
            # initialise policy
            self.learner = PolicyLearner(file_path, device)
            self.initialize_dataset = False
        else:
            os.makedirs(os.path.join(file_path, 'train'))
            os.makedirs(os.path.join(file_path, 'test'))
            self.initialize_dataset = True

        self.dagger_path = os.path.join(file_path,
            datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')+'_dagger')
        self.dagger_models_path = os.path.join(self.dagger_path, 'policies')
        self.dagger_datasets_path = os.path.join(self.dagger_path, 'datasets')
        self.dagger_plots_path = os.path.join(self.dagger_path, 'plots')
        if (not os.path.isdir(self.dagger_models_path)):
            os.makedirs(self.dagger_models_path)
        if (not os.path.isdir(self.dagger_datasets_path)):
            os.makedirs(self.dagger_datasets_path)
        if (not os.path.isdir(self.dagger_plots_path)):
            os.makedirs(self.dagger_plots_path)


        self.p = 0.9 # percentage of times we pick expert over policy
        self.beta = self.p
        self.randomize_policy_choice = False

        # init ros
        rospy.init_node('dagger_orchestrator')
        self.client = actionlib.SimpleActionClient('panda_dagger/collector',
            policy_learning.msg.collect_rolloutAction)

        # subscriber to the mppi cost to track if Dagger is improving the cost
        self.cost_subscriber = rospy.Subscriber("/mppi_data",
                                                Data,
                                                self.cost_callback,
                                                queue_size=10)
        self.data_dict = {
            "stage_cost": [],
            "cost_history": []
        }
        self.mean_costs = []

    def train_torch_model(self, iteration):
        """
        Handles all setup regarding loading files and saving model to train
        a model.
        Args:
            iteration (int): current iteration of the main Dagger loop.
        """
        model_save_path = os.path.join(self.dagger_models_path,
            f'iter_{iteration}')
        self.learner.train()
        self.learner.save_model(model_save_path)

    def collect_initial_dataset(self):
        """
        Collect initial dataset using only the expert.
        """
        n_train = 200
        n_test = 20

        for i in range(n_train):
            dataset_save_dir = os.path.join(self.file_path, 'train',
                f'run_{i}.hdf5')
            goal = policy_learning.msg.collect_rolloutGoal(
                        timeout = 10,
                        use_policy = False,
                        policy_path = "",
                        dataset_path = dataset_save_dir,
                        random_joint_pos = True,
                        random_goal = True)
            self.client.send_goal(goal)
            self.client.wait_for_result(timeout=rospy.Duration(30))

        for i in range(n_test):
            dataset_save_dir = os.path.join(self.file_path, 'test',
                f'run_{i}.hdf5')            
            goal = policy_learning.msg.collect_rolloutGoal(
                        timeout = 10,
                        use_policy = False,
                        policy_path = "",
                        dataset_path = dataset_save_dir,
                        random_joint_pos = True,
                        random_goal = True)
            self.client.send_goal(goal)
            self.client.wait_for_result(timeout=rospy.Duration(30))

    def collect_dataset(self, iteration):
        """
        Handels collecting a dataset at a given iteration with the current policy.
        Args:
            iteration (int): current iteration of the main Dagger loop.
        Returns:
            dataset_save_dir (string): loaction where dataset is saved
        """
        dataset_save_dir_intermed = os.path.join(self.dagger_datasets_path,
            f'iter_{iteration}', 'train')
        if (not os.path.isdir(dataset_save_dir_intermed)):
            os.makedirs(dataset_save_dir_intermed)
        model_load_path = os.path.join(self.dagger_models_path,
            f'iter_{iteration-1}.pt')
        n_runs = 45
        print(f"Collecting data for {n_runs} runs...")
        expert_sum = 0
        policy_sum = 0
        for i in range(n_runs):
            dataset_save_dir = os.path.join(dataset_save_dir_intermed,
                f'run_{i}.hdf5')
            if self.randomize_policy_choice:
                r = bernoulli.rvs(self.beta, size=1)
                use_policy = bool(1-r) # False is to use expert
                if use_policy:
                    policy_sum += 1
                else:
                    expert_sum += 1
                goal = policy_learning.msg.collect_rolloutGoal(
                    timeout = 10,
                    use_policy = use_policy,
                    policy_path = model_load_path,
                    dataset_path = dataset_save_dir,
                    random_joint_pos = True,
                    random_goal = True)
            else:
                goal = policy_learning.msg.collect_rolloutGoal(
                    timeout = 10,
                    use_policy = True,
                    policy_path = model_load_path,
                    dataset_path = dataset_save_dir,
                    random_joint_pos = True,
                    random_goal = True)
            self.client.send_goal(goal)
            self.client.wait_for_result(timeout=rospy.Duration(30))
            if self.client.get_state() == GoalStatus.SUCCEEDED:
                print(f'{i}: Goal Reached')
            elif self.client.get_state() == GoalStatus.PREEMPTED:
                print(f'{i}: Goal not reached')
            else:
                print(f'{i}: Action call failed, state {self.client.get_state()}')
            curr_cost = np.array(self.data_dict["stage_cost"])
            self.mean_costs.append(curr_cost.mean())
            for iter in self.data_dict.values():
                iter.clear()
        print('Done')
        if self.randomize_policy_choice:
            print('Expert percentage: ', expert_sum/(policy_sum+expert_sum))
            print('Actual bias: ', self.beta)
        plotting_path = os.path.join(
            self.dagger_plots_path, f'iter_{iteration}.png')
        self.plot_mean_costs(iteration, plotting_path, n_runs)
        if self.randomize_policy_choice:
            self.beta *= self.p #exponential decrease

        return dataset_save_dir_intermed

    def aggregate_dataset(self, dataset_load_dir):
        self.learner.append_dataset(dataset_load_dir)

    def dagger_loop(self, n_iterations):
        """
        Main loop of the dagger algorithm.
        Args:
            n_iterations: Number of iterations to run the dagger algorithm in total
        """
        print('The time at beginning is: ',
            datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
        print('waiting for action server')
        self.client.wait_for_server()
        if self.initialize_dataset:
            print('Collecting initial dataset')
            self.collect_initial_dataset()
            # set device for policy
            device = 'cuda' if torch.cuda.is_available() else 'cpu'
            # initialise policy
            self.learner = PolicyLearner(self.file_path, device)

        for iter in range(n_iterations):
            if iter == 0:
                self.train_torch_model(iter)
                continue
            save_path = self.collect_dataset(iter)
            self.aggregate_dataset(save_path)
            self.train_torch_model(iter)
            print('After finishing iteration ', iter, ' the time is: ',
                datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
        self.save_cost_statistics()

    def cost_callback(self, data: Data):
        if len(data.weights.array) == 0:
            return
        self.data_dict["stage_cost"].append(data.stage_cost)
        self.data_dict["cost_history"].append(data.rollouts_cost.array)

    def plot_mean_costs(self, dagger_it, save_dir, n_points):
        fig, (ax1, ax2) = plt.subplots(2)
        label = 'Mean Costs in ' + str(dagger_it) + '-th Iteration'
        ax1.plot(self.mean_costs[-n_points:], label=label)
        ax1.set_ylabel('cost []')
        ax1.set_title('Mean Costs')
        ax1.yaxis.grid(True)
        ax1.legend()
        total_points = len(self.mean_costs)
        if  total_points>= 5*n_points:
            ax2.plot(range(total_points-5*n_points, total_points),
                self.mean_costs[-5*n_points:], label='Total')
        else:
            ax2.plot(self.mean_costs, label='Total')
        ax2.yaxis.grid(True)
        ax2.set_ylabel('cost []')
        ax2.set_xlabel('collection sample []')
        ax2.legend()
        plt.savefig(save_dir)

    def save_cost_statistics(self):
        csv_path = os.path.join(self.dagger_plots_path, 'cost.csv')
        with open(csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([f'{self.mean_costs[i]:5.4}' for i in range(len(self.mean_costs))])



if __name__ == "__main__":

    ## Pseudo algorithm structure  (Ross 2011)
    # load first dataset
    # train first policy pi_0
    # --> done here
    # loop
    #   sample trajectories from pi_1 (potentially use mixing with expert)
    #   --> done in ros node
    #   lable visited states with expert
    #   --> done in ros node
    #   # issue: interface is such that expert is loaded into controller --> potential solution: disable via yaml when launching with roslaunch (learning_ratio)
    #   augment data set
    #   # issue: need to augment dataset from cpp -> no, save data file as currently
    #   # possible, then modify datasets in python
    #   --> done in ros node
    #   train classifier
    #   --> done here

    # Dir to dataset
    task_path = os.path.dirname(os.path.realpath(__file__))
    # dataset_name = "Panda_R15_S5_210511_220510"
    dataset_name = "Panda_R15_S5_210522_220000"
    dir_path = os.path.join(task_path, os.pardir, 'data', dataset_name)
    # intiialise dagger object
    dagger = Dagger(dir_path)
    # run dagger
    n_iterations = 50
    dagger.dagger_loop(n_iterations)
