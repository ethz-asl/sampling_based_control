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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "mppi_manipulation/dimensions.h"
#include <kdl/tree.hpp>

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
  void base_twist_callback(const geometry_msgs::TwistConstPtr& msg);
  void object_pose_callback(const geometry_msgs::TransformStampedConstPtr& msg);
  void base_pose_callback(const geometry_msgs::TransformStampedConstPtr& msg);
  void arm_state_callback(const sensor_msgs::JointStateConstPtr& msg);


//  friend std::ostream& operator<<(std::ostream& os, const StateObserver& obs);

 private:
  Eigen::VectorXd state_;

  bool fixed_base_;
  ros::NodeHandle nh_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::Buffer tf2_buffer_;

  // base
  bool base_twist_received_;
  Eigen::Vector3d base_twist_;
  std::mutex base_twist_mutex_;
  ros::Subscriber base_twist_subscriber_;
  std::unique_ptr<ros::CallbackQueue> base_twist_queue_;

  Eigen::Vector3d base_state_;
  geometry_msgs::TransformStamped tf_base_;

  // arm
  Eigen::Matrix<double, 9, 1> dq_;
  Eigen::Matrix<double, 9, 1> q_;  // arm plus the gripper joints

  // debug ros publishing: do not use for realtime control loops
  ros::Publisher base_pose_publisher_;
  ros::Publisher arm_state_publisher_;
  ros::Publisher articulation_state_publisher_;

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