//
// Created by giuseppe on 23.01.21.
//

#pragma once

#include <mutex>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>

#include <manipulation_msgs/State.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#ifdef MELODIC
#include <filters/median.h>
#endif

#ifndef MELODIC
#include <filters/median.hpp>
#endif

#include <mppi_manipulation_royalpanda/butter.h>

namespace manipulation_royalpanda {

/// The scope of this class is to collect info and return the complete mobile
/// robot state

class StateObserver {
 public:
  StateObserver() = delete;
  explicit StateObserver(const ros::NodeHandle& nh);
  ~StateObserver() = default;

 public:
  bool initialize();

 private:
  void base_pose_callback(const nav_msgs::OdometryConstPtr& msg);
  void base_twist_callback(const nav_msgs::OdometryConstPtr& msg);
  void object_pose_callback(const nav_msgs::OdometryConstPtr& msg);
  void arm_state_callback(const sensor_msgs::JointStateConstPtr& msg);
  void wrench_callback(const geometry_msgs::WrenchStampedConstPtr& msg);
  void object_state_callback(const sensor_msgs::JointStateConstPtr& msg);

  void filter_wrench();

public:
  void publish_state();

 private:
  bool simulation_;

  ros::NodeHandle nh_;

  manipulation_msgs::State state_ros_;
  ros::Publisher state_publisher_;

  // time: will be the one received on the arm state
  double time_;
  double previous_publishing_time_;

  // base transforms
  Eigen::Affine3d T_reference_base_;
  Eigen::Affine3d T_world_base_;
  Eigen::Affine3d T_world_reference_;

  // base odometry
  Eigen::Vector3d base_twist_;
  Eigen::Vector3d base_pose_;

  // arm
  Eigen::Matrix<double, 9, 1> dq_;
  Eigen::Matrix<double, 9, 1> q_;  // arm plus the gripper joints

  // debug ros publishing: do not use for realtime control loops
  ros::Publisher base_pose_publisher_;

  geometry_msgs::TwistStamped base_twist_ros_;
  ros::Publisher base_twist_publisher_;

  sensor_msgs::JointState robot_state_;
  ros::Publisher robot_state_publisher_;

  // State subscribers
  ros::Subscriber arm_subscriber_;
  ros::Subscriber base_pose_subscriber_;
  ros::Subscriber base_twist_subscriber_;
  ros::Subscriber object_subscriber_;
  ros::Subscriber wrench_subscriber_;
  ros::Subscriber object_state_subscriber_;

  // Articulation
  double start_relative_angle_;
  double current_relative_angle_;
  bool articulation_first_computation_;
  sensor_msgs::JointState object_state_;
  ros::Publisher object_state_publisher_;

  // Calibration tf
  Eigen::Affine3d T_handle0_shelf_;
  Eigen::Affine3d T_handle0_hinge_;

  // Static tf: the result of calibration (done on first tf)
  Eigen::Affine3d T_world_shelf_;
  Eigen::Affine3d T_hinge_world_;

  // tf required for joint position estimation
  Eigen::Affine3d T_world_handle_;
  Eigen::Affine3d T_hinge_handle_;
  Eigen::Affine3d T_hinge_handle_init_;

  // filter base odometry
  double base_alpha_;

  // wrench
  Eigen::Matrix<double, 12, 1> ext_tau_;

  // get jacobian to convert wrench into joint torques
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
  KDL::JntArray kdl_joints_;
  KDL::Jacobian J_world_ee_;
  KDL::Chain world_to_ee_chain_;

  double wrench_threshold_;
  double wrench_contact_threshold_;

  std::vector<double> wrench_meas_v_; // wrench measured as std vector (input median filter)
  std::vector<double> wrench_medf_; // wrench filtered by median filter

  Eigen::VectorXd wrench_meas_;
  Eigen::VectorXd wrench_filt_;
  Eigen::VectorXd wrench_filt_sensor_;
  geometry_msgs::WrenchStamped wrench_filt_ros_;
  ros::Publisher wrench_filt_publisher_;

  std::array<Butter2, 6> wrench_lp_filters_;
  std::unique_ptr<filters::MultiChannelFilterBase<double>> wrench_median_filter_;

  // tf2_ros
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // contact state
  bool contact_state_ = false;

  // mutex acces to the state
  std::mutex state_mutex_;


};
}  // namespace manipulation_royalpanda
