#! /usr/bin/env  python

import rospy
from geometry_msgs.msg import PoseStamped

# - Translation: [0.547, 0.023, 0.575]
# - Rotation: in Quaternion [0.995, 0.046, -0.085, -0.003]

pose0 = PoseStamped()
pose0.header.frame_id = "world"
pose0.pose.position.x = 0.547
pose0.pose.position.y = 0.023
pose0.pose.position.z = 0.575
pose0.pose.orientation.x = 0.995
pose0.pose.orientation.y = 0.046
pose0.pose.orientation.z = -0.085
pose0.pose.orientation.w = -0.003

# - Translation: [0.506, 0.028, 0.334]
# - Rotation: in Quaternion [0.939, 0.036, 0.343, -0.005]

pose1 = PoseStamped()
pose1.header.frame_id = "world"
pose1.pose.position.x = 0.506
pose1.pose.position.y = 0.028
pose1.pose.position.z = 0.334
pose1.pose.orientation.x = 0.939
pose1.pose.orientation.y = 0.036
pose1.pose.orientation.z = 0.343
pose1.pose.orientation.w = -0.005

# - Translation: [0.241, -0.001, 0.267]
# - Rotation: in Quaternion [0.993, 0.051, -0.107, -0.006]

pose2 = PoseStamped()
pose2.header.frame_id = "world"
pose2.pose.position.x = 0.241
pose2.pose.position.y = -0.001
pose2.pose.position.z = 0.267
pose2.pose.orientation.x = 0.993
pose2.pose.orientation.y = 0.051
pose2.pose.orientation.z = -0.107
pose2.pose.orientation.w = -0.006

# At time 1612539335.589
# - Translation: [0.253, -0.004, 0.546]
# - Rotation: in Quaternion [-0.795, 0.011, 0.097, 0.599]

pose3 = PoseStamped()
pose3.header.frame_id = "world"
pose3.pose.position.x = 0.253
pose3.pose.position.y = -0.004
pose3.pose.position.z = 0.546
pose3.pose.orientation.x = -0.795
pose3.pose.orientation.y = 0.011
pose3.pose.orientation.z = 0.097
pose3.pose.orientation.w = 0.559

# - Translation: [0.325, -0.000, 0.722]
# - Rotation: in Quaternion [0.995, 0.048, -0.082, -0.004]

pose4 = PoseStamped()
pose4.header.frame_id = "world"
pose4.pose.position.x = 0.325
pose4.pose.position.y = 0.0
pose4.pose.position.z = 0.722
pose4.pose.orientation.x = 0.995
pose4.pose.orientation.y = 0.048
pose4.pose.orientation.z = -0.082
pose4.pose.orientation.w = -0.004

#poses = [pose0, pose1, pose2, pose3, pose4]
poses = [pose0]

if __name__ == "__main__":
    rospy.init_node("target_generator")
    pose_publisher = rospy.Publisher("/end_effector_pose_desired",
                                     PoseStamped,
                                     queue_size=10)

    rospy.loginfo("Sleeping for 5 seconds before sending first target.")
    rospy.sleep(5.0)
    for pose in poses:
        pose_publisher.publish(pose)
        rospy.sleep(5.0)
