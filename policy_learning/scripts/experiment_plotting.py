#!/usr/bin/env python3
import os
import datetime
import numpy as np
import matplotlib.pyplot as plt
import csv
import glob
import time

# from learning import PolicyLearner
from record_data import DataRecorder

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from mppi_ros.msg import Data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import policy_learning.msg



class ExperimentPlotter:
    """
    Class to handle the generation of experimental data. It is able to run a
    specified controller either using new random generated IC and goal pose data
    or loads this data from a previous run to ensure that we compare apples to
    apples.
    ===== ATTENTTION =========
    The user has to confirm that params of the controller use to launch the ROS
    controller (i.e. n_rollouts, substeps, learning_ratio, etc.)
    are set correctly himself!
    ==========================
    """
    def __init__(self, root_dir):
        """
        Variables that must be set here:
            self.controller_name    name of controller, will be displayed in plot
            self.mode               mode of experiment collection -> either new
                                    IC and goal poses are generated by controlled
                                    node or these variables are loaded from a
                                    file from a previous run.
                                    In new mode by default there is no policy.
                                    Thus effectively the learned_rollout_ratio is
                                    zero for this case.
            self.experiment_data_load_folder_name
                                    If load mode is chosen this is the dir with
                                    the experiment data.
            self.n_experiment_runs  number of times a goal is visited (only
                                    needs to be set in new mode)
            self.only_use_policy    set false if MPPI should be mixed with learned
                                    policy according to learned_rollout_ratio
                                    value. Set true if only the policy should be
                                    run.
        """
        ### Set here ###
        self.experiment_name = "default"
        self.controller_name = "MPPI"
        self.mode = "from_file"  # new | from_file
        self.n_experiment_runs = 100
        self.only_use_policy = False
        self.experiment_data_load_folder_name = "2021_05_25_14_11_00"
        self.timeout = 10
        self.choose_policy = True
        self.policy_choice = 34 
        ################
        self.root_dir = root_dir
        self.policies_path = os.path.join(self.root_dir, 'policies')
        self.expert_running = True

        # data save dir
        experiments_dir = os.path.join(self.root_dir, os.pardir, os.pardir, os.pardir,
            'experiments')
        if (not os.path.isdir(experiments_dir)):
            os.makedirs(experiments_dir)
        self.experiment_data_save_dir = os.path.join(experiments_dir,
            datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S'))
        os.mkdir(self.experiment_data_save_dir)

        if self.mode == "new":
            self.f = open(os.path.join(self.experiment_data_save_dir, 'log.txt'), mode='w')
            self.f.write('Ran in new mode. No policy is passed by default, thus ignore learned_rollout_ratio.\n')

        elif self.mode == "from_file":
            self.experiment_data_load_dir = os.path.join(experiments_dir,
                self.experiment_data_load_folder_name)
            self.f = open(os.path.join(self.experiment_data_save_dir, 'log.txt'), mode='w')
            # log what policy we are plotting and what the ids are
            self.f.write('Ran in from_file mode.\n')
            self.f.write(f'Took IC and goal data from: {self.experiment_data_load_folder_name}\n')
            self.f.write('Policy taken from the follwing dir: ')
            self.f.write(self.policies_path)
            self.f.write('\n')

        self.read_policy()

        # init ros
        rospy.init_node('experiment_plotter')
        self.client = actionlib.SimpleActionClient('panda_dagger/collector',
            policy_learning.msg.collect_rolloutAction)

        # subscriber to the mppi cost to track if Dagger is improving the cost
        self.cost_subscriber = rospy.Subscriber("/mppi_data",
                                                Data,
                                                self.cost_callback,
                                                queue_size=10000)
        if self.mode == "new":
            self.joint_ic_f = open(
                os.path.join(self.experiment_data_save_dir, 'joint_ic.csv'),
                mode='w')
            self.goal_pose_f = open(
                os.path.join(self.experiment_data_save_dir, 'goal_pose.csv'),
                mode='w')

        elif self.mode == "from_file":
            self.goalS = [] # Goal values list of list
            self.initial_joint_posS = [] # Joint state values list of list
            joint_ic_f_name = os.path.join(
                self.experiment_data_load_dir, 'joint_ic.csv')
            with open(joint_ic_f_name) as joint_ic_f:
                joint_ic_reader = csv.reader(joint_ic_f, delimiter=',')
                for line in joint_ic_reader:
                    self.initial_joint_posS.append(line)

            goal_pose_f_name = os.path.join(
                self.experiment_data_load_dir, 'goal_pose.csv')
            with open(goal_pose_f_name) as goal_pose_f:
                goal_pose_reader = csv.reader(goal_pose_f, delimiter=',')
                for line in goal_pose_reader:
                    self.goalS.append(line)

            if len(self.goalS) == len(self.initial_joint_posS):
                self.n_experiment_runs = len(self.goalS)
            else:
                print("Error occured, lengths not the same!")

        # write current active params to log file
        lrr = rospy.get_param("/panda_dagger/solver/learned_rollout_ratio")
        nr = rospy.get_param("/panda_dagger/solver/rollouts")
        ss = rospy.get_param("/panda_dagger/solver/substeps")
        hrz = rospy.get_param("/panda_dagger/solver/horizon")
        self.f.write('The following params were set:\n')
        self.f.write(f'learned_rollout_ratio: {lrr}\n')
        self.f.write(f'rollouts: {nr}\n')
        self.f.write(f'substeps: {ss}\n')
        self.f.write(f'horizon: {hrz}\n')
        self.f.write('The following are the exp ids to use in plotting:\n')

        # Initialize ros msgs
        self.goal = PoseStamped()
        self.initial_joint_pos = JointState()

        self.run_started = False
        self.goal_reached_time = self.timeout


    # Callback functions
    def cost_callback(self, data: Data):
        if len(data.weights.array) == 0 and not self.run_started:
            return
        else:
            self.run_started = True
            
    def read_policy(self):
        self.policies_cpp = sorted(glob.glob(self.policies_path + '/*.pt'),
            key=lambda f: int(''.join(filter(str.isdigit, f))))
        self.policies_py = sorted(glob.glob(self.policies_path + '/*.pth'),
            key=lambda f: int(''.join(filter(str.isdigit, f))))

        self.final_policy_path = self.policies_cpp[-1]

    def write_pose_information_to_file(self):
        joint_ics = self.initial_joint_pos.position
        string = ''
        for val in joint_ics:
            string += f'{val:.6f},'
        self.joint_ic_f.write(string[:-1])
        self.joint_ic_f.write('\n')
        goal_position = self.goal.pose.position
        goal_orient = self.goal.pose.orientation
        self.goal_pose_f.write(f'{goal_position.x:.6f},{goal_position.y:.6f},{goal_position.z:.6f},{goal_orient.x:.6f},{goal_orient.y:.6f},{goal_orient.z:.6f},{goal_orient.w:.6f}')
        self.goal_pose_f.write('\n')

    def set_pose_information_from_file(self, i):
        # Initial condition
        ic_now = self.initial_joint_posS[i]
        for val in ic_now:
            self.initial_joint_pos.position.append(float(val))
        # Goal pos
        goal_now = self.goalS[i]
        self.goal.header.frame_id = 'world'
        self.goal.pose.position.x = float(goal_now[0])
        self.goal.pose.position.y = float(goal_now[1])
        self.goal.pose.position.z = float(goal_now[2])
        self.goal.pose.orientation.x = float(goal_now[3])
        self.goal.pose.orientation.y = float(goal_now[4])
        self.goal.pose.orientation.z = float(goal_now[5])
        self.goal.pose.orientation.w = float(goal_now[6])

    def close_files(self):
        if self.mode == "new":
            self.joint_ic_f.close()
            self.goal_pose_f.close()
        self.f.close()

    def get_goal(self):
        if self.choose_policy:
            model_load_path = self.policies_cpp[self.policy_choice]
            print('Chose different policy than final policy: ', self.policy_choice)
        else:
            model_load_path = self.final_policy_path
        if self.mode == "new":
            goal = policy_learning.msg.collect_rolloutGoal(
                timeout = self.timeout,
                use_policy = False,
                policy_path = model_load_path,
                dataset_path = None,
                random_goal = True,
                random_joint_pos = True)
        elif self.mode == "from_file":
            goal = policy_learning.msg.collect_rolloutGoal(
                timeout = self.timeout,
                use_policy = self.only_use_policy,
                policy_path = model_load_path,
                dataset_path = None,
                initial_joint_pos = self.initial_joint_pos,
                goal_pose = self.goal,
                random_goal = False,
                random_joint_pos = False)
        else:
            print('No condition reached. Error!')
        return goal

    def action_sender(self, goal, i):
        self.client.send_goal(goal)
        self.client.wait_for_result(timeout=rospy.Duration(240))
        if self.client.get_state() == GoalStatus.SUCCEEDED:
            print(f'{i}: Goal Reached')
            self.goal_reached_time = self.client.get_result().goal_reached_time
            self.goal = self.client.get_result().goal_pose
            self.initial_joint_pos = self.client.get_result().initial_joint_pos
        elif self.client.get_state() == GoalStatus.PREEMPTED:
            print(f'{i}: Goal not reached')
            self.goal_reached_time = self.client.get_result().goal_reached_time
            self.goal = self.client.get_result().goal_pose
            self.initial_joint_pos = self.client.get_result().initial_joint_pos
        else:
            print(f'{i}: Action call failed, state {self.client.get_state()}')
        self.run_started = False

    def run_experiment_loop(self):
        print('waiting for action server')
        self.client.wait_for_server()
        print('found action server')
        for i in range(self.n_experiment_runs):
            recorder = DataRecorder(controller_name=self.controller_name, experiment_id=self.experiment_name)
            if self.mode == "from_file":
                self.set_pose_information_from_file(i)
            goal = self.get_goal()
            self.action_sender(goal, i)
            time.sleep(3)
            success = recorder.postprocess(time_to_goal=self.goal_reached_time)
            if success:
                if self.mode == "new":
                    self.write_pose_information_to_file()
                recorder.save()
                exp_id = recorder.get_experiment_id()
                self.f.write(exp_id)
                self.f.write('\n')
        self.close_files()


if __name__ == "__main__":
    task_path = os.path.dirname(os.path.realpath(__file__))
    dataset_name = "Panda_R20_S10_210522_220000_full"
    dagger_name = "full_no_dagger"
    dir_path = os.path.join(task_path, os.pardir, 'data', dataset_name,
        dagger_name)
    plotter = ExperimentPlotter(dir_path)
    plotter.run_experiment_loop()
