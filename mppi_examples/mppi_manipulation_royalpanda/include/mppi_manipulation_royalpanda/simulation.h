//
// Created by giuseppe on 16.08.21.
//

#pragma once

#pragma once

#include <Eigen/Core>
#include <hardware_interface/robot_hw.h>
#include <franka/robot_state.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/model_base.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <mppi_manipulation/dynamics_ros.h>
#include <ros/ros.h>
#include <cmath>
#include <map>
#include <memory>

namespace manipulation_royalpanda {

class RoyalPandaSim : public hardware_interface::RobotHW{
 public:
  RoyalPandaSim() = delete;
  explicit RoyalPandaSim(const ros::NodeHandle& nh);

  bool init_sim();
  void read_sim(ros::Time time, ros::Duration period);
  void write_sim(ros::Time time, ros::Duration period);
  double get_time_step();
  void publish_ros();

 private:
  bool init_params();
  bool init_dynamics();
  bool init_handles();

 private:
  std::unique_ptr<manipulation::ManipulatorDynamicsRos> dynamics_;

  hardware_interface::JointStateInterface joint_state_if_;
  hardware_interface::EffortJointInterface effort_joint_if_;
  franka_hw::FrankaStateInterface franka_state_if_;
  franka_hw::FrankaModelInterface franka_model_if_;

  franka::RobotState robot_state_;
  std::unique_ptr<franka_hw::ModelBase> model_;

  std::vector<std::string> arm_joint_name_;
  Eigen::VectorXd arm_joint_position_;
  Eigen::VectorXd arm_joint_velocity_;
  Eigen::VectorXd arm_joint_effort_;
  Eigen::VectorXd arm_joint_effort_desired_;

  Eigen::Vector3d base_position_;
  Eigen::Vector3d base_twist_;
  Eigen::Vector3d base_twist_desired_;
  bool contact_state_;
  double object_position_;
  double object_velocity_;
  double tank_state_;

  Eigen::Matrix<double, 12, 1> u_; // compound velocity command (base + arm)

  ros::NodeHandle nh_;
  ros::Publisher state_publisher_;
};

}  // namespace manipulation_royalpanda