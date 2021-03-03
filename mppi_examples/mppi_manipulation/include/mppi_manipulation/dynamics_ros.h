//
// Created by giuseppe on 18.01.21.
//

#pragma once

#include "mppi_manipulation/dynamics.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

namespace manipulation {

class ManipulatorDynamicsRos : public PandaRaisimDynamics {
 public:
  ManipulatorDynamicsRos(const ros::NodeHandle& nh,
                         const std::string& robot_description,
                         const std::string& object_description, const double dt,
                         const bool fixed_base = true);
  ~ManipulatorDynamicsRos() = default;

 public:
  void reset_to_default();
  void publish_ros();

 private:
  ros::NodeHandle nh_;
  ros::Publisher state_publisher_;
  ros::Publisher object_state_publisher_;
  ros::Publisher contact_forces_publisher_;
  ros::Publisher ee_publisher_;
  ros::Publisher handle_publisher_;

  sensor_msgs::JointState joint_state_;
  sensor_msgs::JointState object_state_;
  visualization_msgs::Marker force_marker_;
};

}  // namespace manipulation
