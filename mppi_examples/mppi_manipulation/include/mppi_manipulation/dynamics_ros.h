//
// Created by giuseppe on 18.01.21.
//

#pragma once

#include "mppi_manipulation/dynamics.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

namespace manipulation {

class ManipulatorDynamicsRos : public PandaRaisimDynamics {
 public:
  ManipulatorDynamicsRos(const ros::NodeHandle& nh,
                         const DynamicsParams& params);
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
  ros::Publisher tau_ext_publisher_;
  ros::Publisher power_publisher_;

  sensor_msgs::JointState joint_state_;
  sensor_msgs::JointState object_state_;
  visualization_msgs::Marker force_marker_;
  std_msgs::Float64MultiArray tau_ext_msg_;
  std_msgs::Float64 tank_energy_;
  ros::Publisher tank_energy_publisher_;

  Eigen::VectorXd ff_tau_;
  Eigen::VectorXd integral_term_;
};

}  // namespace manipulation
