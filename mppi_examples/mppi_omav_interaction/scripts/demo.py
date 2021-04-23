#! /usr/bin/env  python

import rospy
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    rospy.init_node("reference_node")
    rospy.loginfo("Give new reference in 3 seconds.")

    reference_publisher = rospy.Publisher("/mppi_pose_desired",
                                          PoseStamped,
                                          queue_size=10)

    reference_pose = PoseStamped()
    reference_pose.header.frame_id = "odom"
    reference_pose.pose.position.x = 10.0
    reference_pose.pose.position.y = 10.0
    reference_pose.pose.position.z = 10.0
    reference_pose.pose.orientation.w = 1.0
    reference_pose.pose.orientation.x = 0.0
    reference_pose.pose.orientation.y = 0.0
    reference_pose.pose.orientation.z = 0.0
    rospy.sleep(3.0)

    rospy.loginfo("Setting reference as target")
    reference_publisher.publish(reference_pose)
