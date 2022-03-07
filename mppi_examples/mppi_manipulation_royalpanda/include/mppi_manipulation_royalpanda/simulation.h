//
// Created by giuseppe on 16.08.21.
//

#pragma once

#pragma once

#include <franka/robot_state.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/model_base.h>
#include <geometry_msgs/WrenchStamped.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <mppi_manipulation/dynamics_ros.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <Eigen/Core>
#include <cmath>

#ifdef MELODIC
#include <filters/median.h>
#endif

#ifndef MELODIC
#include <filters/median.hpp>
#endif

#include <map>
#include <memory>

namespace manipulation_royalpanda {

class RoyalPandaSim : public hardware_interface::RobotHW{
 public:
  RoyalPandaSim() = delete;
  explicit RoyalPandaSim(const ros::NodeHandle& nh);

  bool init_sim();
  void read_sim(ros::Time time, ros::Duration period);
  void advance_sim(ros::Time time, ros::Duration period);
  double get_time_step();
  void publish_ros();

  Eigen::VectorXd get_state() const;
  void print_state();

 private:
  bool init_params();
  bool init_dynamics();
  void init_handles();
  void init_publishers();

  void base_twist_cmd_callback(const geometry_msgs::TwistConstPtr& msg);
  void apply_external_force_callback(const geometry_msgs::WrenchConstPtr& msg);
  bool e_stop_cb(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);

 private:
  std::unique_ptr<manipulation::ManipulatorDynamicsRos> dynamics_;

  hardware_interface::JointStateInterface joint_state_if_;
  hardware_interface::EffortJointInterface effort_joint_if_;
  hardware_interface::VelocityJointInterface velocity_joint_if_;
  franka_hw::FrankaStateInterface franka_state_if_;
  franka_hw::FrankaModelInterface franka_model_if_;

  franka::RobotState robot_state_;
  std::unique_ptr<franka_hw::ModelBase> model_;

  std::vector<std::string> base_joint_name_;
  std::vector<std::string> arm_joint_name_;
  std::vector<std::string> finger_joint_name_;

  Eigen::VectorXd arm_joint_position_;
  Eigen::VectorXd arm_joint_velocity_;
  Eigen::VectorXd arm_joint_effort_;
  Eigen::VectorXd arm_joint_effort_desired_;

  Eigen::Vector3d base_position_;
  Eigen::Vector3d base_twist_;
  Eigen::Vector3d base_effort_;
  Eigen::Vector3d base_twist_cmd_;
  Eigen::Vector3d base_twist_cmd_shared_;  // the command written using shared
                                           // memory (sim only)
  bool contact_state_;
  double object_position_;
  double object_velocity_;

  Eigen::Matrix<double, 12, 1> u_; // compound velocity command (base + arm)

  ros::NodeHandle nh_;

  std::string base_odom_topic_;
  std::string base_twist_topic_;
  std::string base_twist_cmd_topic_;
  std::string handle_odom_topic_;
  std::string arm_state_topic_;
  std::string object_state_topic_;
  std::string finger_state_topic_;
  std::string wrench_topic_;

  nav_msgs::Odometry base_pose_odom_;  // from vicon
  nav_msgs::Odometry base_twist_odom_; // from ridgeback
  nav_msgs::Odometry handle_odom_;     // from vicon
  sensor_msgs::JointState arm_state_;
  sensor_msgs::JointState finger_state_;
  sensor_msgs::JointState object_state_;  // sim only

  ros::Publisher base_pose_publisher_;
  ros::Publisher base_twist_publisher_;
  ros::Publisher arm_state_publisher_;
  ros::Publisher finger_state_publisher_;
  ros::Publisher object_pose_publisher_;
  ros::Publisher object_state_publisher_;
  ros::Publisher wrench_publisher_;
  ros::Subscriber base_twist_cmd_subscriber_;
  ros::Subscriber external_force_subscriber_;

  // External wrench variables
  Eigen::VectorXd wrench_;
  geometry_msgs::WrenchStamped wrench_ros_;

  std::vector<double> wrench_temp_;
  std::vector<double> wrench_filtered_;
  std::unique_ptr<filters::MultiChannelFilterBase<double>> wrench_filter_;

  Eigen::VectorXd tau_ext_base_arm_;
  Eigen::MatrixXd ee_jacobian_;
  Eigen::MatrixXd ee_jacobian_transpose_pinv_;
  Eigen::Matrix<double, 6, 1> ee_wrench_;

  // external force from user
  Eigen::Vector3d user_force_;

  // sim time measurements
  double simulation_step_;

  // e stop
  bool e_stop_;
  ros::ServiceServer e_stop_service_;
};

}  // namespace manipulation_royalpanda