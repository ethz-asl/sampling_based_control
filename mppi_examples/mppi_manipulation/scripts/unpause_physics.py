#! /usr/bin/env  python

import rospy

import time
from std_srvs.srv import Empty

if __name__ == "__main__":
    rospy.init_node("unpause_physics_node")
    delay_sec = rospy.get_param("~delay_sec")
    rospy.loginfo("Waiting {} sec before unpausing the physics.")
    # Do not rely on simulated time: not running because physics is paused
    time.sleep(delay_sec)

    service_client = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
    try:
        service_client.wait_for_service(5.0)
        service_client.call()
    except rospy.ROSException as exc:
        rospy.logerr(exc)
