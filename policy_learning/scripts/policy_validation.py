#!/usr/bin/env python3
import os
import datetime
import numpy as np
import matplotlib.pyplot as plt
import csv
import glob

# from learning import PolicyLearner

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from mppi_ros.msg import Data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import policy_learning.msg



class PolicyValidater:
    """
    Class to handle validation of a policy. It is able to run the expert and the
    policy using the same conditions and then compare the performance to each
    other according to cost metrics from the expert.
    ===== ATTENTTION =========
    The user has to confirm that params are set correctly himself!
    ==========================
    """
    def __init__(self, root_dir):
        self.root_dir = root_dir
        self.policies_path = os.path.join(self.root_dir, 'policies')
        self.expert_running = True
        # plotting dir
        val_dir = os.path.join(self.root_dir, os.pardir, os.pardir, os.pardir,
            'validation')
        if (not os.path.isdir(val_dir)):
            os.makedirs(val_dir)
        self.plotting_dir = os.path.join(val_dir,
            datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')+'_plots')
        os.mkdir(self.plotting_dir)
        self.n_validation_runs = 50

        # log what policy we are plotting
        f = open(os.path.join(self.plotting_dir, 'log.txt'), mode='w')
        f.write('Policy taken from the follwing dir:\n')
        f.write(self.policies_path)
        f.close()


        self.read_policy()

        # init ros
        rospy.init_node('policy_validation')
        self.client = actionlib.SimpleActionClient('panda_validation/validator',
            policy_learning.msg.collect_rolloutAction)

        # subscriber to the mppi cost to track if Dagger is improving the cost
        self.cost_subscriber = rospy.Subscriber("/mppi_data",
                                                Data,
                                                self.cost_callback,
                                                queue_size=10)
        self.goal_pose_subscriber = rospy.Subscriber("/end_effector_pose_desired",
                                                    PoseStamped,
                                                    self.goal_pose_callback,
                                                    queue_size=10)
        self.initial_joint_position_subscriber = rospy.Subscriber("/initial_state",
                                                    JointState,
                                                    self.initial_joint_position_callback,
                                                    queue_size=10)
        self.expert_data_dict = {
            "stage_cost": [],
            "cost_history": []
        }
        self.policy_data_dict = {
            "stage_cost": [],
            "cost_history": []
        }
        self.expert_mean_costs = []
        self.policy_mean_costs = []
        self.run_started = False
        self.expert_goal = None # PoseStamped
        self.expert_initial_joint_pos = None # JointState

    # Callback functions
    def cost_callback(self, data: Data):
        if len(data.weights.array) == 0 and not self.run_started:
            return
        else:
            self.run_started = True
        if self.expert_running:
            self.expert_data_dict["stage_cost"].append(data.stage_cost)
            self.expert_data_dict["cost_history"].append(data.rollouts_cost.array)
        else:
            self.policy_data_dict["stage_cost"].append(data.stage_cost)
            self.policy_data_dict["cost_history"].append(data.rollouts_cost.array)

    def goal_pose_callback(self, pose: PoseStamped):
        self.expert_goal = pose
        print('Received expert goal pose')

    def initial_joint_position_callback(self, state: JointState):
        self.expert_initial_joint_pos = state
        print('Recieved expert joint initial position')

    def read_policy(self):
        self.policies_cpp = sorted(glob.glob(self.policies_path + '/*.pt'),
            key=lambda f: int(''.join(filter(str.isdigit, f))))
        self.policies_py = sorted(glob.glob(self.policies_path + '/*.pth'),
            key=lambda f: int(''.join(filter(str.isdigit, f))))

        self.final_policy_path = self.policies_cpp[-1]

    def get_expert_goal(self):
        self.expert_running = True
        self.expert_initial_joint_pos_set = False
        self.expert_goal_set = False
        model_load_path = self.final_policy_path
        goal = policy_learning.msg.collect_rolloutGoal(
            timeout = 10,
            use_policy = False,
            policy_path = model_load_path,
            dataset_path = None,
            validate = False)
        return goal

    def get_policy_goal(self):
        self.expert_running = False
        model_load_path = self.final_policy_path
        goal = policy_learning.msg.collect_rolloutGoal(
            timeout = 10,
            use_policy = True,
            policy_path = model_load_path,
            dataset_path = None,
            validate = True,
            initial_joint_pos = self.expert_initial_joint_pos,
            goal_pose = self.expert_goal)
        return goal

    def action_sender(self, goal, i):
        self.client.send_goal(goal)
        self.client.wait_for_result(timeout=rospy.Duration(30))
        if self.client.get_state() == GoalStatus.SUCCEEDED:
            print(f'{i}: Goal Reached')
        elif self.client.get_state() == GoalStatus.PREEMPTED:
            print(f'{i}: Goal not reached')
        else:
            print(f'{i}: Action call failed, state {self.client.get_state()}')
        self.run_started = False
        if self.expert_running:
            curr_cost = np.array(self.expert_data_dict["stage_cost"])
            self.expert_mean_costs.append(curr_cost.mean())
        else:
            curr_cost = np.array(self.policy_data_dict["stage_cost"])
            self.policy_mean_costs.append(curr_cost.mean())



    def run_validation_loop(self):
        print('waiting for action server')
        self.client.wait_for_server()
        print('found action server')
        for i in range(self.n_validation_runs):
            expert_goal = self.get_expert_goal()
            self.action_sender(expert_goal, i)
            policy_goal = self.get_policy_goal()
            self.action_sender(policy_goal, i)
            self.plot_cost_trajectory(i)
            self.clear_data_containers()

        self.plot_mean_costs()


    def plot_cost_trajectory(self, iter):
        fig, ax = plt.subplots()
        ax.plot(self.expert_data_dict["stage_cost"], label="Expert")
        ax.plot(self.policy_data_dict["stage_cost"], label="Policy")
        ax.set_ylabel('Cost []')
        ax.set_xlabel('Rollout iteration[]')
        ax.yaxis.grid(True)
        ax.set_title('Comparison of Expert to Policy over One Rollout')
        ax.legend()
        save_dir = os.path.join(self.plotting_dir, f'iter_{iter}.png')
        plt.savefig(save_dir)
        plt.close(fig)

    def plot_mean_costs(self):
        fig, ax = plt.subplots()
        ax.plot(self.expert_mean_costs, label="Expert")
        ax.plot(self.policy_mean_costs, label="Policy")
        ax.set_ylabel('Cost []')
        ax.set_xlabel('Test iteration []')
        ax.yaxis.grid(True)
        ax.set_title('Comparison of Mean Costs')
        ax.legend()
        save_dir = os.path.join(self.plotting_dir, 'means.png')
        plt.savefig(save_dir)
        plt.close(fig)

    def clear_data_containers(self):
        for iter in self.expert_data_dict.values():
            iter.clear()
        for iter in self.policy_data_dict.values():
            iter.clear()

if __name__ == "__main__":
    task_path = os.path.dirname(os.path.realpath(__file__))
    dataset_name = "Panda_R15_S5_210511_220510"
    dagger_name = "2021_05_17_22_53_28_dagger"
    dir_path = os.path.join(task_path, os.pardir, 'data', dataset_name,
        dagger_name)
    verifier = PolicyValidater(dir_path)
    verifier.run_validation_loop()
