/*!
 * @file    dynamics_ros.cpp
 * @author  Matthias Studiger
 * @date    10.04.2021
 * @version 1.0
 * @brief   description
 */

#include "mppi_omav_velocity/dynamics_ros.h"

namespace omav_velocity {
OMAVVelocityDynamicsRos::OMAVVelocityDynamicsRos(
    const ros::NodeHandle &nh, const std::string &robot_description,
    const std::string &object_description, const double dt)
    : OMAVVelocityDynamics(robot_description, object_description, dt) {
  vis_publisher_ =
      nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  goal_publisher_ = nh_.advertise<visualization_msgs::Marker>("goal_marker", 0);
  obstacle_publisher_ =
      nh_.advertise<visualization_msgs::Marker>("obstacle_marker", 0);
  object_state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("/object/joint_state", 10);

  omav_marker_.header.frame_id = "odom";
  omav_marker_.id = 0;
  omav_marker_.action = visualization_msgs::Marker::ADD;
  omav_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
  omav_marker_.mesh_resource =
      "package://mppi_omav_velocity/data/meshes/ouzel_inspection_full.dae";
  omav_marker_.scale.x = 1.0;
  omav_marker_.scale.y = 1.0;
  omav_marker_.scale.z = 1.0;
  omav_marker_.color.a = 1.0;
  omav_marker_.color.r = 1.0;
  omav_marker_.color.g = 0.0;
  omav_marker_.color.b = 0.0;

  goal_marker_.header.frame_id = "odom";
  goal_marker_.id = 1;
  goal_marker_.action = visualization_msgs::Marker::ADD;
  goal_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
  goal_marker_.mesh_resource =
      "package://mppi_omav_velocity/data/meshes/ouzel_inspection_full.dae";
  goal_marker_.scale.x = 1.0;
  goal_marker_.scale.y = 1.0;
  goal_marker_.scale.z = 1.0;
  goal_marker_.color.a = 1.0;
  goal_marker_.color.r = 0.0;
  goal_marker_.color.g = 1.0;
  goal_marker_.color.b = 0.0;
  goal_marker_.header.stamp = ros::Time::now();
  goal_marker_.pose.position.x = 10.0;
  goal_marker_.pose.position.y = 10.0;
  goal_marker_.pose.position.z = 10.0;
  goal_marker_.pose.orientation.w = 1.0;
  goal_marker_.pose.orientation.x = 0.0;
  goal_marker_.pose.orientation.y = 0.0;
  goal_marker_.pose.orientation.z = 0.0;

  obstacle_marker_.header.frame_id = "odom";
  obstacle_marker_.id = 2;
  obstacle_marker_.action = visualization_msgs::Marker::ADD;
  obstacle_marker_.type = visualization_msgs::Marker::CYLINDER;
  obstacle_marker_.scale.x = 2.0;
  obstacle_marker_.scale.y = 2.0;
  obstacle_marker_.scale.z = 10.0;
  obstacle_marker_.color.a = 1.0;
  obstacle_marker_.color.r = 1.0;
  obstacle_marker_.color.g = 0.0;
  obstacle_marker_.color.b = 0.0;
  obstacle_marker_.header.stamp = ros::Time::now();
  obstacle_marker_.pose.position.x = 5.0;
  obstacle_marker_.pose.position.y = 5.0;
  obstacle_marker_.pose.position.z = 5.0;
  obstacle_marker_.pose.orientation.w = 1.0;
  obstacle_marker_.pose.orientation.x = 0.0;
  obstacle_marker_.pose.orientation.y = 0.0;
  obstacle_marker_.pose.orientation.z = 0.0;

  object_state_.name = {"articulation_joint"};
  object_state_.position.resize(1);
}
void OMAVVelocityDynamicsRos::reset_to_default() {
  x_.setZero();
  x_(9) = 1.0;
  reset(x_);
  ROS_INFO_STREAM("Reset simulation ot default value: " << x_.transpose());
}
void OMAVVelocityDynamicsRos::publish_ros() {

  object_state_.header.stamp = ros::Time::now();
  object_state_.position[0] = x_(13);
  object_state_publisher_.publish(object_state_);

  vis_publisher_.publish(omav_marker_);
  goal_publisher_.publish(goal_marker_);
  obstacle_publisher_.publish(obstacle_marker_);
}
} // namespace omav_velocity
