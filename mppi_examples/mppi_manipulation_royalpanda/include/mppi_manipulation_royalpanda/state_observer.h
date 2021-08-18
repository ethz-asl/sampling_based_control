//
// Created by giuseppe on 23.01.21.
//

#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <manipulation_msgs/State.h>
#include <manipulation_msgs/StateRequest.h>

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
  void update();
  void publish();

 private:
  void base_pose_callback(const nav_msgs::OdometryConstPtr& msg);
  void base_twist_callback(const nav_msgs::OdometryConstPtr& msg);
  void object_pose_callback(const nav_msgs::OdometryConstPtr& msg);
  void arm_state_callback(const sensor_msgs::JointStateConstPtr& msg);
  void wrench_callback(const geometry_msgs::WrenchStampedConstPtr& msg);

  bool state_request_cb(manipulation_msgs::StateRequestRequest& req,
                        manipulation_msgs::StateRequestResponse& res);

 private:
  bool base_pose_received_;
  bool base_twist_received_;
  bool object_pose_received_;
  bool arm_state_received_;

  ros::NodeHandle nh_;

  manipulation_msgs::State state_ros_;
  ros::Publisher state_publisher_;

  // time: will be the one received on the arm state
  double time_;

  // tank
  double tank_state_;

  // base transforms
  Eigen::Affine3d T_reference_base_;
  Eigen::Affine3d T_world_base_;
  Eigen::Affine3d T_world_reference_;

  // base odometry
  double base_twist_time_;
  Eigen::Vector3d base_twist_;
  double base_pose_time_;
  Eigen::Vector3d base_pose_;

  // arm
  double arm_state_time_;
  Eigen::Matrix<double, 9, 1> dq_;
  Eigen::Matrix<double, 9, 1> q_;  // arm plus the gripper joints

  // debug ros publishing: do not use for realtime control loops
  ros::Publisher base_pose_publisher_;

  geometry_msgs::TwistStamped base_twist_ros_;
  ros::Publisher base_twist_publisher_;

  sensor_msgs::JointState robot_state_;
  ros::Publisher robot_state_publisher_;

  // vicon/simulation subscribers
  ros::Subscriber object_pose_subscriber_;
  ros::Subscriber base_pose_subscriber_;
  ros::Subscriber base_twist_subscriber_;
  ros::Subscriber arm_state_subscriber_;

  ///// Articulation
  // articulation
  double object_pose_time_;
  double previous_time_;
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

  // this is only to enforce determinism in simulation
  ros::ServiceServer sync_state_service_;

  // wrench
  Eigen::Matrix<double, 12, 1> ext_tau_;
};
}  // namespace royalpanda
