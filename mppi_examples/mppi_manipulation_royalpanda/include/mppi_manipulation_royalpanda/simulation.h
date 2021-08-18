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
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/WrenchStamped.h>
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

  Eigen::VectorXd get_state() const;
  void print_state();

 private:
  bool init_params();
  bool init_dynamics();
  void init_handles();
  void init_publishers();

  void base_twist_cmd_callback(const geometry_msgs::TwistConstPtr& msg);

 private:
  std::unique_ptr<manipulation::ManipulatorDynamicsRos> dynamics_;

  hardware_interface::JointStateInterface joint_state_if_;
  hardware_interface::EffortJointInterface effort_joint_if_;
  franka_hw::FrankaStateInterface franka_state_if_;
  franka_hw::FrankaModelInterface franka_model_if_;

  franka::RobotState robot_state_;
  std::unique_ptr<franka_hw::ModelBase> model_;

  std::vector<std::string> arm_joint_name_;
  std::vector<std::string> finger_joint_name_;

  Eigen::VectorXd arm_joint_position_;
  Eigen::VectorXd arm_joint_velocity_;
  Eigen::VectorXd arm_joint_effort_;
  Eigen::VectorXd arm_joint_effort_desired_;

  Eigen::Vector3d base_position_;
  Eigen::Vector3d base_twist_;
  Eigen::Vector3d base_twist_cmd_;
  Eigen::Vector3d base_effort_;
  bool contact_state_;
  double object_position_;
  double object_velocity_;
  double tank_state_;

  Eigen::Matrix<double, 12, 1> u_; // compound velocity command (base + arm)

  ros::NodeHandle nh_;

  std::string base_odom_topic_;
  std::string base_twist_topic_;
  std::string base_twist_cmd_topic_;
  std::string handle_odom_topic_;
  std::string arm_state_topic_;
  std::string finger_state_topic_;
  std::string wrench_topic_;

  nav_msgs::Odometry base_pose_odom_;  // from vicon
  nav_msgs::Odometry base_twist_odom_; // from ridgeback
  nav_msgs::Odometry handle_odom_;     // from vicon
  sensor_msgs::JointState arm_state_;
  sensor_msgs::JointState finger_state_;
  geometry_msgs::WrenchStamped wrench_;

  ros::Publisher base_pose_publisher_;
  ros::Publisher base_twist_publisher_;
  ros::Publisher arm_state_publisher_;
  ros::Publisher finger_state_publisher_;
  ros::Publisher object_pose_publisher_;
  ros::Publisher wrench_publisher_;
  ros::Subscriber base_twist_cmd_subscriber_;

  // External force computation
  Eigen::VectorXd tau_ext_base_arm_;
  Eigen::MatrixXd ee_jacobian_;
  Eigen::MatrixXd ee_jacobian_transpose_pinv_;
  Eigen::Matrix<double, 6, 1> ee_wrench_;
};

}  // namespace manipulation_royalpanda