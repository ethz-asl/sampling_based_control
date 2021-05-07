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

# Setup paths for dataset
parser = argparse.ArgumentParser(description='Run data collection for pole cart')
parser.add_argument('name',
                    type=str,
                    help='name of the dataset')
parser.add_argument('--n_runs', '-n',
                    type=int,
                    default=50,
                    help='Number of rollouts')
args = parser.parse_args()

data_path = os.path.join(RosPack().get_path('policy_learning'), 'data')
dataset_name = args.name + datetime.now().strftime('_%y%m%d_%H%M%S') 
dataset_path = os.path.join(data_path, dataset_name)
os.mkdir(dataset_path)
csv_path = os.path.join(dataset_path, 'value_log.csv')

with open(csv_path, 'a', newline='') as f:
    writer = csv.writer(f)
    writer.writerow([f'joint_{i+1}' for i in range(7)])

# initial conditions
limits_lower = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
limits_upper = np.array([2.8973, 1.7628, 2.8973, 0.0698, 2.8973, 3.7525, 2.8973])
nominal_position = np.array([0, -0.5, 0, -1.8, 0, 1, 0.7])
std_dev = np.minimum(limits_upper-nominal_position, nominal_position-limits_lower) / 4

# target pose
def get_pose():
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

# Init publishing node
rospy.init_node('target_pub', anonymous=True)
target_pose_pub = rospy.Publisher('/end_effector_pose_desired', PoseStamped, queue_size=10)
target_pose = PoseStamped()
target_pose.header.frame_id = 'world'

obstacle_pose_pub = rospy.Publisher('/obstacle', PoseStamped, queue_size=10)
obstacle_pose = PoseStamped()
obstacle_pose.header.frame_id = 'world'
obstacle_pose.pose.position = Point(1000., 1000., 1000.,) # ignore obstacle...

for i in range(args.n_runs):
  joint_state = np.random.normal(nominal_position, std_dev)
  joint_state = np.clip(joint_state, limits_lower, limits_upper)
  joint_state_string = np.array2string(joint_state, separator=', ', precision=4, max_line_width=1000)

  with open(csv_path, 'a', newline='') as f:
    writer = csv.writer(f)
    writer.writerow([f'{j:1.4}' for j in joint_state])

  output_path = os.path.join(dataset_path, f'run_{i}.hdf5')
  
  panda_cli_args = ['mppi_panda', 'panda_data_collection.launch', 
                    f'learner_output_path:={output_path}',
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

  obstacle_pose_pub.publish(obstacle_pose)
  p, q = get_pose()
  target_pose.pose.position = Point(*p)
  target_pose.pose.orientation = Quaternion(*q)
  target_pose_pub.publish(target_pose)

  parent.spin()
  parent.shutdown()


