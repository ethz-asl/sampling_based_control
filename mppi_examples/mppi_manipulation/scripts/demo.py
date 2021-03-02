#! /usr/bin/env  python

import sys
import rospy
import tf2_ros
from std_msgs.msg import Int64
from geometry_msgs.msg import PoseStamped

if __name__ == "__main__":
    rospy.init_node("demo_node")
    rospy.loginfo("Starting the demo in 3 seconds.")
    rospy.sleep(3.0)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    mode_publisher = rospy.Publisher("/mode", Int64, queue_size=10)
    pose_publisher = rospy.Publisher("/end_effector_pose_desired",
                                     PoseStamped,
                                     queue_size=10)

    try:
        transform = tfBuffer.lookup_transform('world', 'panda_grasp',
                                              rospy.Time(),
                                              rospy.Duration(3.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        rospy.logerr("Failed to get panda grasp transform")
        sys.exit()

    default_pose = PoseStamped()
    trans = transform.transform.translation
    rot = transform.transform.rotation

    default_pose.header.frame_id = "world"
    default_pose.pose.position.x = trans.x
    default_pose.pose.position.y = trans.y
    default_pose.pose.position.z = trans.z
    default_pose.pose.orientation.w = rot.w
    default_pose.pose.orientation.x = rot.x
    default_pose.pose.orientation.y = rot.y
    default_pose.pose.orientation.z = rot.z

    go_to_handle_mode = Int64()
    go_to_handle_mode.data = 1

    open_mode = Int64()
    open_mode.data = 2

    default_mode = Int64()
    default_mode.data = 0

    rospy.loginfo("Setting current pose as target")
    pose_publisher.publish(default_pose)
    rospy.sleep(1.0)

    mode_publisher.publish(go_to_handle_mode)
    rospy.loginfo("Reaching the handle!")
    rospy.sleep(15.0)

    mode_publisher.publish(open_mode)
    rospy.loginfo("Opening the door!")
    rospy.sleep(15.0)

    mode_publisher.publish(default_mode)
    pose_publisher.publish(default_pose)
    rospy.loginfo("Going back to default pose!")
    rospy.sleep(15.0)

    rospy.loginfo("Done :)")
