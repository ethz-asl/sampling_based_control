#!/usr/bin/env python3
import rospy
from mppi_ros.recorder import DataRecorder

if __name__ == "__main__":
    rospy.init_node("record_data")
    #experiment_id = rospy.get_param("~experiment_id")
    recorder = DataRecorder()
    rospy.spin()

    recorder.postprocess()
    recorder.save()