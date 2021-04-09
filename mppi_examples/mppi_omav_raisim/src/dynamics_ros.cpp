/*!
 * @file    dynamics_ros.cpp
 * @author  Matthias Studiger
 * @date    22.03.2021
 * @version 1.0
 * @brief   description
 */

#include "mppi_omav_raisim/dynamics_ros.h"

namespace omav_raisim {
OMAVRaisimDynamicsRos::OMAVRaisimDynamicsRos(
    const ros::NodeHandle &nh, const std::string &robot_description,
    const double dt)
    : OMAVRaisimDynamics(robot_description, dt) {
  vis_publisher_ =
      nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  goal_publisher_ = nh_.advertise<visualization_msgs::Marker>("goal_marker", 0);
  obstacle_publisher_ = nh_.advertise<visualization_msgs::Marker>("obstacle_marker", 0);


  omav_marker_.header.frame_id = "odom";
  omav_marker_.id = 0;
  omav_marker_.action = visualization_msgs::Marker::ADD;
  omav_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
  omav_marker_.mesh_resource =
      "package://mppi_omav_raisim/data/meshes/ouzel_inspection_full.dae";
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
      "package://mppi_omav_raisim/data/meshes/ouzel_inspection_full.dae";
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

}
void OMAVRaisimDynamicsRos::reset_to_default() {
  x_.setZero();
  x_(9) = 1.0;
  reset(x_);
  ROS_INFO_STREAM("Reset simulation ot default value: " << x_.transpose());
}
void OMAVRaisimDynamicsRos::publish_ros() {
  // Update robot state visualization
  omav_marker_.header.stamp = ros::Time::now();
  omav_marker_.pose.position.x = x_(16);
  omav_marker_.pose.position.y = x_(17);
  omav_marker_.pose.position.z = x_(18);
  omav_marker_.pose.orientation.w = x_(9);
  omav_marker_.pose.orientation.x = x_(10);
  omav_marker_.pose.orientation.y = x_(11);
  omav_marker_.pose.orientation.z = x_(12);

  vis_publisher_.publish(omav_marker_);
  goal_publisher_.publish(goal_marker_);
  obstacle_publisher_.publish(obstacle_marker_);
}
}  // namespace omav_raisim
