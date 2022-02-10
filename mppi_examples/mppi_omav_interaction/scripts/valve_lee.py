#! /usr/bin/env  python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int64, Float64


def cost_callback(data):
    if data.data < 100:
        reference_pose.header.frame_id = "odom"
        reference_pose.pose.position.x = 0.0
        reference_pose.pose.position.y = 0.0
        reference_pose.pose.position.z = 0.5
        reference_pose.pose.orientation.w = 1.0
        reference_pose.pose.orientation.x = 0.0
        reference_pose.pose.orientation.y = 0.0
        reference_pose.pose.orientation.z = 0.0
        rospy.loginfo("Task sufficiently good executed.")
        reference_publisher.publish(reference_pose)

def set_init_pose(pos_mode=0):
    reference_pose = PoseStamped()
    reference_pose.header.stamp = rospy.get_rostime()
    reference_pose.header.frame_id = "world"    
    if pos_mode == 0:
        reference_pose.pose.position.x = 0.05
        reference_pose.pose.position.y = 0.92
        reference_pose.pose.position.z = 1.3
        reference_pose.pose.orientation.w = 0
        reference_pose.pose.orientation.x = 0.0
        reference_pose.pose.orientation.y = 0.0
        reference_pose.pose.orientation.z = 1.0
    else:
        reference_pose.pose.position.x = 1.0
        reference_pose.pose.position.y = 0.7
        reference_pose.pose.position.z = 0.78
        reference_pose.pose.orientation.w = 0.707
        reference_pose.pose.orientation.x = 0.0
        reference_pose.pose.orientation.y = 0.0
        reference_pose.pose.orientation.z = -0.707
    return reference_pose

def set_retreat_pose(pos_mode=0):
    reference_pose = PoseStamped()
    reference_pose.header.stamp = rospy.get_rostime()
    reference_pose.header.frame_id = "world"    
    if pos_mode == 0:
        reference_pose.pose.position.x = 0.5  # Use this parameter to set the ref position or velocity of the valve.
        reference_pose.pose.position.y = 0.92
        reference_pose.pose.position.z = 1.5
        reference_pose.pose.orientation.w = 0.0
        reference_pose.pose.orientation.x = 0.0
        reference_pose.pose.orientation.y = 0.0
        reference_pose.pose.orientation.z = 1.0
    else:
        reference_pose.pose.position.x = 1.0
        reference_pose.pose.position.y = 1.4
        reference_pose.pose.position.z = 0.78
        reference_pose.pose.orientation.w = 0.707
        reference_pose.pose.orientation.x = 0.0
        reference_pose.pose.orientation.y = 0.0
        reference_pose.pose.orientation.z = -0.707
    return reference_pose

if __name__ == "__main__":
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
    pos_mode = 0
    reference_pose = set_init_pose(pos_mode)

    mode.data = 0.0

    rospy.loginfo("Setting reference as target")
    reference_publisher.publish(reference_pose)
    mode_publisher.publish(mode)

    rospy.sleep(5.0)
    mode.data = 1.0

    rospy.loginfo("Starting Interaction")
    mode_publisher.publish(mode)

    rospy.sleep(15.0)
    reference_pose = set_retreat_pose(pos_mode)
    mode.data = 0.0
    rospy.loginfo("Setting reference as target")
    reference_publisher.publish(reference_pose)
    mode_publisher.publish(mode)
