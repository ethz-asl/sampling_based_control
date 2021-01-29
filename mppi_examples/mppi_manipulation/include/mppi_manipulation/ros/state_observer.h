//
// Created by giuseppe on 23.01.21.
//

#pragma once

#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

namespace manipulation {

/// The scope of this class is to collect info and return the complete mobile robot state

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


//  friend std::ostream& operator<<(std::ostream& os, const StateObserver& obs);

 private:
  Eigen::VectorXd state_;

  bool fixed_base_;
  ros::NodeHandle nh_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::Buffer tf2_buffer_;

  // base
  Eigen::Affine3d T_reference_base_;
  Eigen::Affine3d T_world_base_;
  Eigen::Affine3d T_world_reference_;
  Eigen::Vector3d base_twist_;
  ros::Subscriber base_twist_subscriber_;

  Eigen::Vector3d base_state_;
  geometry_msgs::TransformStamped tf_base_;

  // arm
  Eigen::Matrix<double, 9, 1> dq_;
  Eigen::Matrix<double, 9, 1> q_;  // arm plus the gripper joints

  // debug ros publishing: do not use for realtime control loops
  ros::Publisher base_pose_publisher_;

  sensor_msgs::JointState robot_state_;
  ros::Publisher robot_state_publisher_;

  // vicon subscribers
  ros::Subscriber object_pose_subscriber_;
  ros::Subscriber base_pose_subscriber_;
  ros::Subscriber arm_state_subscriber_;

  ///// Articulation
  // articulation
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

  geometry_msgs::PoseStamped T_world_shelf_ros_;
  ros::Publisher object_root_publisher_;

  KDL::Tree robot_kinematics;
  KDL::Tree object_kinematics;
  KDL::Chain robot_chain;

  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_robot_solver;

  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped T_world_shelf_ros;

};
}  // namespace manipulation

//std::ostream& operator<<(std::ostream& os, const manipulation::StateObserver& obs);

//std::ostream& operator<<(std::ostream& os, const manipulation::StateObserver& obs){
//  os << "\n";
//  os << "=========================================================\n";
//  os << "                        State                            \n";
//  os << "=========================================================\n";
//  if (!obs.fixed_base_) {
//    os << "Base position:  " << obs.base_state_.transpose() << std::endl;
//    os << "Base twist:     " << obs.base_twist_.transpose() << std::endl;
//  }
//  os << "Joint position: " << obs.q_.transpose() << std::endl;
//  os << "Joint velocity: " << obs.dq_.transpose() << std::endl;
//  os << "Theta:          " << obs.object_state_.position[0] << std::endl;
//  os << "Theta dot:      " << obs.object_state_.velocity[0] << std::endl;
//  os << "x: " << std::setprecision(2) << obs.state_.transpose() << std::endl;
//  return os;
//};