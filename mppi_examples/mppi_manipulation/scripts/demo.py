#! /usr/bin/env  python

import sys
import rospy
from std_msgs.msg import Int64


if __name__ == "__main__":
	rospy.init_node("demo_node")
	rospy.loginfo("Starting the demo in 3 seconds.")
	rospy.sleep(3.0)

	mode_publisher = rospy.Publisher("/mode", Int64, queue_size=10)
	
	ans = input("Have you selected the default pose from the marker [y/n]")
	if ans != 'y':
	    rospy.loginfo("Exiting")
	    sys.exit(0)
	      
	go_to_handle_mode = Int64()
	go_to_handle_mode.data = 1

	open_mode = Int64()
	open_mode.data = 2

	default_mode = Int64()
	default_mode.data = 0


	mode_publisher.publish(go_to_handle_mode)
	rospy.loginfo("Reaching the handle!")
	rospy.sleep(15.0)

	mode_publisher.publish(open_mode)
	rospy.loginfo("Opening the door!")
	rospy.sleep(15.0)
	
	mode_publisher.publish(default_mode)
	rospy.loginfo("Going back to default pose!")
	rospy.sleep(15.0)

	rospy.loginfo("Done :)")
	

