import os
import argparse
from datetime import datetime
import numpy as np
import csv
import math

import rospy
from rospkg import RosPack
import roslaunch
from geometry_msgs.msg import PoseStamped, Point, Quaternion

class DatasetCollectionPanda:
    """
    Should eventually inherit an be implemenation of abstract class but for now
    only implemnation of data collection for panda.
    """
    def __init__(self, args, dagger=False, save_path=None, n_runs=None,
        model_path=None):
        if not dagger:
            # Setup paths for dataset if not dagger
            self.n_runs = args.n_runs
            data_path = os.path.join(RosPack().get_path('policy_learning'), 'data')
            dataset_name = args.name + datetime.now().strftime('_%y%m%d_%H%M%S')
            self.dataset_path = os.path.join(data_path, dataset_name)
            os.mkdir(self.dataset_path)

        if dagger:
            self.n_runs = n_runs
            self.dataset_path = save_path
            self.model_path = model_path
            os.mkdir(self.dataset_path)

        self.csv_path = os.path.join(self.dataset_path, 'value_log.csv')
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([f'joint_{i+1}' for i in range(7)])

        self.panda_init()

    def panda_init(self):
        # initial conditions
        self.limits_lower = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
        self.limits_upper = np.array([2.8973, 1.7628, 2.8973, 0.0698, 2.8973, 3.7525, 2.8973])
        self.nominal_position = np.array([0, -0.5, 0, -1.8, 0, 1, 0.7])
        self.std_dev = np.minimum(self.limits_upper-self.nominal_position,
            self.nominal_position-self.limits_lower) / 4

        # Init publishing node
        rospy.init_node('target_pub', anonymous=True)
        self.target_pose_pub = rospy.Publisher('/end_effector_pose_desired', PoseStamped, queue_size=10)
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = 'world'

        self.obstacle_pose_pub = rospy.Publisher('/obstacle', PoseStamped, queue_size=10)
        self.obstacle_pose = PoseStamped()
        self.obstacle_pose.header.frame_id = 'world'
        self.obstacle_pose.pose.position = Point(1000., 1000., 1000.,) # ignore obstacle...

    def run_collection(self):
        for i in range(self.n_runs):
          joint_state = np.random.normal(self.nominal_position, self.std_dev)
          joint_state = np.clip(joint_state, self.limits_lower, self.limits_upper)
          joint_state_string = np.array2string(joint_state, separator=', ', precision=4, max_line_width=1000)

          with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([f'{j:1.4}' for j in joint_state])

          output_path = os.path.join(self.dataset_path, f'run_{i}.hdf5')

          panda_cli_args = ['mppi_panda', 'panda_dagger.launch',
                            f'learner_output_path:={output_path}',
                            f'torchscript_model_path:={self.model_path}',
                            f"initial_condition_vector:={joint_state_string}"]

          uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
          roslaunch.configure_logging(uuid)
          roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(panda_cli_args)[0]
          parent = roslaunch.parent.ROSLaunchParent(uuid,
                                                    [(roslaunch_file, panda_cli_args[2:])],
                                                    force_required=True,
                                                    timeout=60)
          parent.start()
          rospy.sleep(5)

          self.obstacle_pose_pub.publish(self.obstacle_pose)
          p, q = self.get_pose()
          self.target_pose.pose.position = Point(*p)
          self.target_pose.pose.orientation = Quaternion(*q)
          self.target_pose_pub.publish(self.target_pose)

          parent.spin()
          parent.shutdown()

    # target pose
    def get_pose(self):
      # generate true random rotation as explained here http://planning.cs.uiuc.edu/node198.html
      u = np.random.uniform(size=3)
      quat = np.array([math.sqrt(1-u[0]) * math.sin(2*math.pi * u[1]),
                       math.sqrt(1-u[0]) * math.cos(2*math.pi * u[1]),
                       math.sqrt(u[0])   * math.sin(2*math.pi * u[2]),
                       math.sqrt(u[0])   * math.cos(2*math.pi * u[2])])

      max_rad = 0.7
      min_rad = 0.2
      center = np.array([0, 0, 0.4])
      while True:
        position = np.random.uniform(low=-max_rad, high=max_rad, size=3)
        if min_rad <= np.linalg.norm(position) <= max_rad:
          position += center
          if position[0] > 0 and position[2] > 0:
            break
      return position, quat
