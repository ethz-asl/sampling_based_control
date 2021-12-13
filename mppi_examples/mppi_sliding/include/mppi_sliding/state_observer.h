//
// Created by boyang on 09.12.21.
//

#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>

#include <manipulation_msgs/State.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

namespace manipulation_panda {

/// The scope of this class is to collect info and return the complete panda
/// robot state

class StateObserver {
 public:
  StateObserver() = delete;
  explicit StateObserver(const ros::NodeHandle& nh);
  ~StateObserver() = default;

 public:
  bool initialize();

 private:
  bool init_ros();
  // void base_pose_callback(const nav_msgs::OdometryConstPtr& msg);
  // void base_twist_callback(const nav_msgs::OdometryConstPtr& msg);
  void object_pose_callback(const nav_msgs::OdometryConstPtr& msg);
  void arm_state_callback(const sensor_msgs::JointStateConstPtr& msg);
  void wrench_callback(const geometry_msgs::WrenchStampedConstPtr& msg);
  void object_state_callback(const sensor_msgs::JointStateConstPtr& msg);

  void message_filter_cb(const sensor_msgs::JointStateConstPtr& arm_state,
                          const sensor_msgs::JointStateConstPtr& object_state);

  void message_filter_cb_sim(
      const sensor_msgs::JointStateConstPtr& arm_state,
      const nav_msgs::OdometryConstPtr& base_pose,
      const nav_msgs::OdometryConstPtr& base_twist,
      const sensor_msgs::JointStateConstPtr& object_state,
      const geometry_msgs::WrenchStampedConstPtr& wrench){};

 private:
  bool simulation_;

  ros::NodeHandle nh_;

  manipulation_msgs::State state_ros_;
  ros::Publisher state_publisher_;

  // time: will be the one received on the arm state
  double time_;
  double previous_publishing_time_;

  // tank
  double tank_state_;

  // base transforms
  // Eigen::Affine3d T_reference_base_;
  // Eigen::Affine3d T_world_base_;
  // Eigen::Affine3d T_world_reference_;

  // // base odometry
  // Eigen::Vector3d base_twist_;
  // Eigen::Vector3d base_pose_;

  // arm
  Eigen::Matrix<double, 9, 1> dq_;
  Eigen::Matrix<double, 9, 1> q_;  // arm plus the gripper joints

  // debug ros publishing: do not use for realtime control loops
  // ros::Publisher base_pose_publisher_;

  // geometry_msgs::TwistStamped base_twist_ros_;
  // ros::Publisher base_twist_publisher_;

  sensor_msgs::JointState robot_state_;
  ros::Publisher robot_state_publisher_;

  ros::Publisher table_state_publisher_;
  sensor_msgs::JointState table_state_;

  geometry_msgs::TransformStamped table_trans;
  geometry_msgs::TransformStamped object_state_trans;
  tf2_ros::TransformBroadcaster broadcaster;

  ros::Subscriber object_pose_subscriber_;

  ros::Subscriber arm_state_subscriber_;
  ros::Subscriber wrench_subscriber_;

  // message filter
  bool exact_sync_;
  message_filters::Subscriber<sensor_msgs::JointState> arm_sub_;
  // message_filters::Subscriber<nav_msgs::Odometry> base_pose_sub_;
  // message_filters::Subscriber<nav_msgs::Odometry> base_twist_sub_;
  message_filters::Subscriber<sensor_msgs::JointState> object_sub_;
  message_filters::Subscriber<sensor_msgs::JointState> obj_state_sub_;  // sim
  message_filters::Subscriber<geometry_msgs::WrenchStamped> wrench_sub_;

  // clang-format off
  using ExactPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::JointState,
                                                                nav_msgs::Odometry,
                                                                nav_msgs::Odometry,
                                                                nav_msgs::Odometry,
                                                                geometry_msgs::WrenchStamped>;

  using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState,                                                                          
                                                                            sensor_msgs::JointState>;

  using ExactPolicySim = message_filters::sync_policies::ExactTime<sensor_msgs::JointState,
                                                                   nav_msgs::Odometry,
                                                                   nav_msgs::Odometry,
                                                                   sensor_msgs::JointState,
                                                                   geometry_msgs::WrenchStamped>;

  using ApproximatePolicySim = message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState,                                                                          
                                                                               sensor_msgs::JointState>;

  std::unique_ptr<message_filters::Synchronizer<ExactPolicy>> exact_message_filter_;
  std::unique_ptr<message_filters::Synchronizer<ApproximatePolicy>> approx_message_filter_;
  std::unique_ptr<message_filters::Synchronizer<ExactPolicySim>> exact_message_filter_sim_;
  std::unique_ptr<message_filters::Synchronizer<ApproximatePolicySim>> approx_message_filter_sim_;
  // clang-format off

  ///// Articulation
  // articulation
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
  //double base_alpha_;

  // wrench
  Eigen::Matrix<double, 12, 1> ext_tau_;

  // get jacobian to convert wrench into joint torques
  std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
  KDL::JntArray kdl_joints_;
  KDL::Jacobian J_world_ee_;
  Eigen::Matrix<double, 6, 1> wrench_eigen_;
  KDL::Chain world_to_ee_chain_;

  // tf2_ros
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // contact state
  bool contact_state_ = false;
};
}  // namespace manipulation_panda
