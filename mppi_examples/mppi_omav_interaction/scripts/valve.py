#! /usr/bin/env  python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int64, Float64
from scipy.spatial.transform import Rotation as Rot
import sys

def set_init_pose(pos_mode):
    reference_pose = PoseStamped()
    reference_pose.header.stamp = rospy.get_rostime()
    reference_pose.header.frame_id = "world"    
    if pos_mode == 'lee':
        r = Rot.from_euler('y',15,degrees=True).as_quat()
        reference_pose.pose.position.x = 1.06
        reference_pose.pose.position.y = 0.35
        reference_pose.pose.position.z = 1.3
        reference_pose.pose.orientation.w = r[3]
        reference_pose.pose.orientation.x = r[0]
        reference_pose.pose.orientation.y = r[1]
        reference_pose.pose.orientation.z = r[2]
    elif pos_mode =='sim':
        r = Rot.from_euler('y',15,degrees=True).as_quat()
        reference_pose.pose.position.x = 0.4
        reference_pose.pose.position.y = 0.0
        reference_pose.pose.position.z = 1.1
        reference_pose.pose.orientation.w = r[3]
        reference_pose.pose.orientation.x = r[0]
        reference_pose.pose.orientation.y = r[1]
        reference_pose.pose.orientation.z = r[2]
    else:
        raise ValueError('Wrong position mode.')
    return reference_pose

def set_retreat_pose(pos_mode):
    reference_pose = PoseStamped()
    reference_pose.header.stamp = rospy.get_rostime()
    reference_pose.header.frame_id = "world"    
    if pos_mode == 'lee':
        reference_pose.pose.position.x = 0.8  # Use this parameter to set the ref position or velocity of the valve.
        reference_pose.pose.position.y = 0.9
        reference_pose.pose.position.z = 1.3
        reference_pose.pose.orientation.w = 1.0
        reference_pose.pose.orientation.x = 0.0
        reference_pose.pose.orientation.y = 0.0
        reference_pose.pose.orientation.z = 0.0
    elif pos_mode =='sim':
        reference_pose.pose.position.x = 0.4
        reference_pose.pose.position.y = 0.0
        reference_pose.pose.position.z = 1.1
        reference_pose.pose.orientation.w = 1
        reference_pose.pose.orientation.x = 0.0
        reference_pose.pose.orientation.y = 0.0
        reference_pose.pose.orientation.z = 0.0
    else:
        raise ValueError('Wrong position mode.')
    return reference_pose

if __name__ == "__main__":
    if len(sys.argv) == 1 or (sys.argv[1] != 'lee' and sys.argv[1] != 'sim'):
        raise ValueError("Wrong mode, choose lee or sim.")
        exit()
    rospy.init_node("reference_node")
    # rospy.loginfo("Set new reference in 3 seconds.")

    reference_publisher = rospy.Publisher("/mppi_pose_desired",
                                          PoseStamped,
                                          queue_size=10)
    mode_publisher = rospy.Publisher("/mppi_omav_mode", Int64, queue_size=10)
    object_reference_publisher = rospy.Publisher("/mppi_object_desired",
                                                 PoseStamped,
                                                 queue_size=10)

    mode = Int64()

    rospy.sleep(1.0)
    pos_mode = sys.argv[1]
    reference_pose = set_init_pose(pos_mode)

    mode.data = 0.0

    rospy.loginfo("Setting reference as target")
    reference_publisher.publish(reference_pose)
    mode_publisher.publish(mode)

    rospy.sleep(5.0)
    mode.data = 1.0

    rospy.loginfo("Starting Interaction")
    mode_publisher.publish(mode)

    rospy.sleep(10.0)
    reference_pose = set_retreat_pose(pos_mode)
    mode.data = 0.0
    rospy.loginfo("Setting reference as target")
    reference_publisher.publish(reference_pose)
    mode_publisher.publish(mode)
