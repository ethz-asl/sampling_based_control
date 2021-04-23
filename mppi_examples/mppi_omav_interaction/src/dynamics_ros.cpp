/*!
 * @file    dynamics_ros.cpp
 * @author  Matthias Studiger
 * @date    10.04.2021
 * @version 1.0
 * @brief   description
 */

#include "mppi_omav_interaction/dynamics_ros.h"

namespace omav_interaction {
OMAVVelocityDynamicsRos::OMAVVelocityDynamicsRos(
    const ros::NodeHandle &nh, const std::string &robot_description,
    const std::string &object_description, const double dt)
    : OMAVVelocityDynamics(robot_description, object_description, dt) {
  vis_publisher_ =
      nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  goal_publisher_ = nh_.advertise<visualization_msgs::Marker>("goal_marker", 0);
  obstacle_publisher_ =
      nh_.advertise<visualization_msgs::Marker>("obstacle_marker", 0);
  object_publisher_ =
      nh_.advertise<visualization_msgs::Marker>("object_marker", 0);

  omav_marker_.header.frame_id = "odom";
  omav_marker_.id = 0;
  omav_marker_.action = visualization_msgs::Marker::ADD;
  omav_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
  omav_marker_.mesh_resource =
      "package://mppi_omav_interaction/data/meshes/ouzel_inspection_full.dae";
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
      "package://mppi_omav_interaction/data/meshes/ouzel_inspection_full.dae";
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

  object_marker_.header.frame_id = "odom";
  object_marker_.id = 3;
  object_marker_.action = visualization_msgs::Marker::ADD;
  object_marker_.type = visualization_msgs::Marker::CUBE;
  object_marker_.scale.x = 0.3;
  object_marker_.scale.y = 0.3;
  object_marker_.scale.z = 0.3;
  object_marker_.color.a = 1.0;
  object_marker_.color.r = 1.0;
  object_marker_.color.g = 1.0;
  object_marker_.color.b = 1.0;
  object_marker_.header.stamp = ros::Time::now();
  object_marker_.pose.position.x = 1.0;
  object_marker_.pose.position.y = 0.0;
  object_marker_.pose.position.z = 0.0;
  object_marker_.pose.orientation.w = 1.0;
  object_marker_.pose.orientation.x = 0.0;
  object_marker_.pose.orientation.y = 0.0;
  object_marker_.pose.orientation.z = 0.0;
}

void OMAVVelocityDynamicsRos::reset_to_default() {
  x_.setZero();
  x_(9) = 1.0;
  reset(x_);
  ROS_INFO_STREAM("Reset simulation ot default value: " << x_.transpose());
}

void OMAVVelocityDynamicsRos::publish_ros() {
  // Update robot state visualization
  omav_marker_.header.stamp = ros::Time::now();
  omav_marker_.pose.position.x = x_(0);
  omav_marker_.pose.position.y = x_(1);
  omav_marker_.pose.position.z = x_(2);
  omav_marker_.pose.orientation.w = x_(3);
  omav_marker_.pose.orientation.x = x_(4);
  omav_marker_.pose.orientation.y = x_(5);
  omav_marker_.pose.orientation.z = x_(6);

  object_marker_.header.stamp = ros::Time::now();
  object_marker_.pose.position.x = x_(13);
  object_marker_.pose.position.y = x_(14);
  object_marker_.pose.position.z = x_(15);
  object_marker_.pose.orientation.w = x_(16);
  object_marker_.pose.orientation.x = x_(17);
  object_marker_.pose.orientation.y = x_(18);
  object_marker_.pose.orientation.z = x_(19);

  vis_publisher_.publish(omav_marker_);
  goal_publisher_.publish(goal_marker_);
  obstacle_publisher_.publish(obstacle_marker_);
  object_publisher_.publish(object_marker_);
}
} // namespace omav_velocity
