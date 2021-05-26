/*!
 * @file    dynamics_ros.h
 * @author  Matthias Studiger
 * @date    10.04.2021
 * @version 1.0
 * @brief   description
 */

#pragma once

#include "mppi_omav_velocity/dynamics.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <visualization_msgs/Marker.h>

namespace omav_velocity {

class OMAVVelocityDynamicsRos : public OMAVVelocityDynamics {
public:
  OMAVVelocityDynamicsRos(const ros::NodeHandle &nh,
                          const std::string &robot_description,
                          const std::string &object_description,
                          const double dt);
  ~OMAVVelocityDynamicsRos() = default;

public:
  void reset_to_default();
  void publish_ros();

private:
  ros::NodeHandle nh_;
  // State Publisher will be unnecessary once we use it in the omav structure,
  // only used for the rviz visualization.
  ros::Publisher vis_publisher_;
  ros::Publisher goal_publisher_;
  ros::Publisher obstacle_publisher_;
  ros::Publisher object_state_publisher_;

  sensor_msgs::JointState object_state_;

  visualization_msgs::Marker goal_marker_;
  visualization_msgs::Marker omav_marker_;
  visualization_msgs::Marker obstacle_marker_;
};
} // namespace omav_velocity
