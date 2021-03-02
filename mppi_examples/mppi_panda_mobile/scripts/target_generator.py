#! /usr/bin/env  python

import rospy
from mppi_ros.msg import Data
from geometry_msgs.msg import PoseStamped

pose0 = PoseStamped()
pose0.header.frame_id = "world"
pose0.pose.position.x = 1.127
pose0.pose.position.y = 0.079
pose0.pose.position.z = 0.451
pose0.pose.orientation.x = 0.995
pose0.pose.orientation.y = 0.045
pose0.pose.orientation.z = -0.084
pose0.pose.orientation.w = -0.008

pose1 = PoseStamped()
pose1.header.frame_id = "world"
pose1.pose.position.x = 0.174
pose1.pose.position.y = -1.222
pose1.pose.position.z = 0.165
pose1.pose.orientation.x = 0.998
pose1.pose.orientation.y = 0.002
pose1.pose.orientation.z = 0.044
pose1.pose.orientation.w = 0.041

pose2 = PoseStamped()
pose2.header.frame_id = "world"
pose2.pose.position.x = -0.565
pose2.pose.position.y = -1.570
pose2.pose.position.z = 0.414
pose2.pose.orientation.x = -0.691
pose2.pose.orientation.y = 0.030
pose2.pose.orientation.z = -0.721
pose2.pose.orientation.w = -0.031

pose3 = PoseStamped()
pose3.header.frame_id = "world"
pose3.pose.position.x = -1.149
pose3.pose.position.y = -0.863
pose3.pose.position.z = 0.068
pose3.pose.orientation.x = -0.646
pose3.pose.orientation.y = -0.207
pose3.pose.orientation.z = 0.689
pose3.pose.orientation.w = -0.256

pose4 = PoseStamped()
pose4.header.frame_id = "world"
pose4.pose.position.x = 0.325
pose4.pose.position.y = 0.0
pose4.pose.position.z = 0.722
pose4.pose.orientation.x = 0.995
pose4.pose.orientation.y = 0.048
pose4.pose.orientation.z = -0.082
pose4.pose.orientation.w = -0.004

pose5 = PoseStamped()
pose5.header.frame_id = "world"
pose5.pose.position.x = -0.845
pose5.pose.position.y = -0.419
pose5.pose.position.z = 0.272
pose5.pose.orientation.x = 0.511
pose5.pose.orientation.y = 0.609
pose5.pose.orientation.z = -0.413
pose5.pose.orientation.w = -0.444

pose6 = PoseStamped()
pose6.header.frame_id = "world"
pose6.pose.position.x = -0.722
pose6.pose.position.y = 1.372
pose6.pose.position.z = 0.302
pose6.pose.orientation.x = -0.637
pose6.pose.orientation.y = -0.168
pose6.pose.orientation.z = 0.125
pose6.pose.orientation.w = 0.742

poses = [pose0, pose1, pose2, pose3, pose4]

idx = 0
trigger = False


def data_callback(data: Data):
    global idx
    global trigger
    idx += 1
    if idx % 500 == 0:
        rospy.loginfo("Triggering new pose: idx = " + str(idx))
        trigger = True


if __name__ == "__main__":
    rospy.init_node("target_generator")
    rospy.loginfo("\n\n\n Targtet generator \n\n\n")
    pose_publisher = rospy.Publisher("/end_effector_pose_desired",
                                     PoseStamped,
                                     queue_size=10)
    data_subscriber = rospy.Subscriber("/mppi_data",
                                       Data,
                                       data_callback,
                                       queue_size=10)

    experiment_id = rospy.get_param("~experiment_id")
    if experiment_id == "panda_mobile_kin_multi_target":
        for pose in poses:
            while not trigger:
                continue

            rospy.loginfo("Sending new pose!")
            pose_publisher.publish(pose)
            trigger = False

    elif experiment_id == "panda_mobile_kin_single_target":
        pose_publisher.publish(pose5)
        rospy.sleep(5.0)
    elif experiment_id == "panda_mobile_kin_nh_single_target":
        pose_publisher.publish(pose6)
        rospy.sleep(5.0)
    else:
        rospy.loginfo("Unknown experiment id: {}".format(experiment_id))
