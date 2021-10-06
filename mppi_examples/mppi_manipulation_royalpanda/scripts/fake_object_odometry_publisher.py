#! /usr/env/bin python
import rospy
from nav_msgs.msg import Odometry

rospy.init_node("fake_odometry_publisher")

pub = rospy.Publisher("/shelf_door/vrpn_client/estimated_odometry", Odometry)
msg = Odometry()
msg.pose.pose.position.x = 3.0
msg.pose.pose.position.y = 2.0
msg.pose.pose.position.z = 0.23

msg.twist.twist.linear.x = 0.0
msg.twist.twist.linear.y = 0.0
msg.twist.twist.linear.z = 0.0

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    msg.header.stamp = rospy.get_rostime()
    pub.publish(msg)
    rate.sleep()
