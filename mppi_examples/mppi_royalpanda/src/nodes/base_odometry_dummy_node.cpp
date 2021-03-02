//
// Created by giuseppe on 23.01.21.
//

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Geometry>

int main(int argc, char** argv) {
  ros::init(argc, argv, "base_twist_publisher");
  ros::NodeHandle nh("~");

  ros::Publisher twist_talker = nh.advertise<nav_msgs::Odometry>(
      "/ridgeback_velocity_controller/odom", 1);
  ros::Publisher base_tf_publisher = nh.advertise<nav_msgs::Odometry>(
      "/panda_base/vrpn_client/estimated_odometry", 1);
  ros::Publisher handle_tf_publisher = nh.advertise<nav_msgs::Odometry>(
      "/shelf_door/vrpn_client/estimated_odometry", 1);
  ros::Publisher arm_state_publisher =
      nh.advertise<sensor_msgs::JointState>("/franka/state", 1);

  nav_msgs::Odometry twist;
  twist.twist.twist.linear.x = 0.01;
  twist.twist.twist.linear.y = 0.001;
  twist.twist.twist.angular.z = 0.1;

  nav_msgs::Odometry base_pose;
  base_pose.header.frame_id = "world";
  base_pose.child_frame_id = "ridgeback";
  base_pose.pose.pose.position.x = 0;
  base_pose.pose.pose.position.y = 0;
  base_pose.pose.pose.position.z = 0;
  base_pose.pose.pose.orientation.x = 0.0;
  base_pose.pose.pose.orientation.y = 0.0;
  base_pose.pose.pose.orientation.z = 0.0;
  base_pose.pose.pose.orientation.w = 1.0;
  double yaw = 0;

  nav_msgs::Odometry handle_pose;
  handle_pose.header.frame_id = "world";
  handle_pose.child_frame_id = "handle_link";
  handle_pose.pose.pose.position.x = 0.554;
  handle_pose.pose.pose.position.y = 0.05;
  handle_pose.pose.pose.position.z = 0.45;
  handle_pose.pose.pose.orientation.x = 0.0;
  handle_pose.pose.pose.orientation.y = 0.0;
  handle_pose.pose.pose.orientation.z = 1.0;
  handle_pose.pose.pose.orientation.w = 0.0;

  sensor_msgs::JointState arm_state;
  arm_state.header.frame_id = "world";
  for (size_t i = 0; i < 7; i++) {
    arm_state.name.push_back("panda_joint" + std::to_string(i + 1));
    arm_state.position.push_back(0.0);
    arm_state.velocity.push_back(0.0);
    arm_state.effort.push_back(0.0);
  }
  arm_state.name.push_back("panda_finger_joint1");
  arm_state.position.push_back(0.0);
  arm_state.velocity.push_back(0.0);
  arm_state.effort.push_back(0.0);

  arm_state.name.push_back("panda_finger_joint2");
  arm_state.position.push_back(0.0);
  arm_state.velocity.push_back(0.0);
  arm_state.effort.push_back(0.0);

  ros::Rate rate(100);
  while (ros::ok()) {
    twist_talker.publish(twist);

    base_pose.header.stamp = ros::Time::now();
    base_pose.pose.pose.position.x += twist.twist.twist.linear.x * 0.01;
    base_pose.pose.pose.position.y += twist.twist.twist.linear.y * 0.01;
    yaw += twist.twist.twist.angular.z * 0.01;
    Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    base_pose.pose.pose.orientation.x = q.x();
    base_pose.pose.pose.orientation.y = q.y();
    base_pose.pose.pose.orientation.z = q.z();
    base_pose.pose.pose.orientation.w = q.w();
    base_pose.header.stamp = ros::Time::now();
    base_tf_publisher.publish(base_pose);

    handle_pose.header.stamp = ros::Time::now();
    handle_tf_publisher.publish(handle_pose);

    arm_state.header.stamp = ros::Time::now();
    arm_state_publisher.publish(arm_state);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}