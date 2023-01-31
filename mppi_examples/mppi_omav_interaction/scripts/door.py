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


if __name__ == "__main__":
    rospy.init_node("reference_node")
    rospy.loginfo("Set new reference in 3 seconds.")

    reference_publisher = rospy.Publisher("/mppi_pose_desired",
                                          PoseStamped,
                                          queue_size=10)
    mode_publisher = rospy.Publisher("/mppi_omav_mode", Int64, queue_size=10)
    object_reference_publisher = rospy.Publisher("/mppi_object_desired",
                                                 PoseStamped,
                                                 queue_size=10)

    reference_pose = PoseStamped()
    mode = Int64()
    object_pose = PoseStamped()

    rospy.sleep(3.0)
    reference_pose.header.frame_id = "world"
    reference_pose.pose.position.x = -0.1
    reference_pose.pose.position.y = 0.1
    reference_pose.pose.position.z = 1.2
    reference_pose.pose.orientation.w = 0.68
    reference_pose.pose.orientation.x = 0.002
    reference_pose.pose.orientation.y = 0.003
    reference_pose.pose.orientation.z = -0.7328431
    mode.data = 0

    rospy.loginfo("Setting reference as target")
    reference_publisher.publish(reference_pose)
    mode_publisher.publish(mode)

    rospy.sleep(10.0)
    object_pose.header.frame_id = "world"
    object_pose.pose.position.x = 1.5708
    object_pose.pose.position.y = 0
    object_pose.pose.position.z = 0
    object_pose.pose.orientation.w = 1.0
    object_pose.pose.orientation.x = 0.0
    object_pose.pose.orientation.y = 0.0
    object_pose.pose.orientation.z = 0.0
    mode.data = 1

    rospy.loginfo("Setting Object Target")
    object_reference_publisher.publish(object_pose)
    mode_publisher.publish(mode)

    rospy.sleep(20.0)
    reference_pose.header.frame_id = "world"
    reference_pose.pose.position.x = 0.0
    reference_pose.pose.position.y = 1.0
    reference_pose.pose.position.z = 1.0
    reference_pose.pose.orientation.w = 0.68
    reference_pose.pose.orientation.x = 0.002
    reference_pose.pose.orientation.y = 0.003
    reference_pose.pose.orientation.z = -0.7328431
    mode.data = 0
    rospy.loginfo("Setting reference as target")
    reference_publisher.publish(reference_pose)
    mode_publisher.publish(mode)
