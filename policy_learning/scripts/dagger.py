#!/usr/bin/env python3

import os
import datetime
import torch

from learning import PolicyLearner

import rospy
import actionlib
import policy_learning.msg

class Dagger:
    """
    Class to handle the generic dataset aggregation loop with virtual methods
    to be completed in the example which are implementation specific.
    """

    def __init__(self, file_path):
        # handle path to initial dataset or (TODO) flag to start from fresh (data collection)
        self.dagger_path = os.path.join(file_path,
            datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')+'_dagger')
        self.dagger_models_path = os.path.join(self.dagger_path, 'policies')
        self.dagger_datasets_path = os.path.join(self.dagger_path, 'datasets')
        if (not os.path.isdir(self.dagger_models_path)):
            os.makedirs(self.dagger_models_path)
        if (not os.path.isdir(self.dagger_datasets_path)):
            os.makedirs(self.dagger_datasets_path)

        # set device for policy
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        # initialise policy
        self.learner = PolicyLearner(file_path, device)

        # init ros
        rospy.init_node('dagger_orchestrator')
        self.client = actionlib.SimpleActionClient('panda_dagger/collector',
            policy_learning.msg.collect_rolloutAction)

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
        model_load_path = os.path.join(self.dagger_models_path,
            f'iter_{iteration-1}.pt')
        n_runs = 2
        for i in range(n_runs):
            dataset_save_dir = os.path.join(dataset_save_dir_intermed,
                f'run_{i}.hdf5')
            goal = policy_learning.msg.collect_rolloutGoal(
                timeout = 60,
                use_policy = False,
                policy_path = model_load_path,
                dataset_path = dataset_save_dir)
            self.client.send_goal(goal)
            self.client.wait_for_result()
            if self.client.get_result():
                print('Server was successfull')


        return dataset_save_dir

    def aggregate_dataset(self, dataset_load_dir):
        self.learner.append_dataset(dataset_load_dir)

    def dagger_loop(self, n_iterations):
        """
        Main loop of the dagger algorithm.
        Args:
            n_iterations: Number of iterations to run the dagger algorithm in total
        """
        print('waiting for action server')
        self.client.wait_for_server()
        for iter in range(n_iterations):
            if iter == 0:
                self.train_torch_model(iter)
                continue
            save_path = self.collect_dataset(iter)
            self.aggregate_dataset(save_path)
            self.train_torch_model(iter)


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
    dataset_name = "Panda_R15_S5_210511_220510"
    dir_path = os.path.join(task_path, os.pardir, 'data', dataset_name)
    # intiialise dagger object
    dagger = Dagger(dir_path)
    # run dagger
    n_iterations = 10
    dagger.dagger_loop(n_iterations)
