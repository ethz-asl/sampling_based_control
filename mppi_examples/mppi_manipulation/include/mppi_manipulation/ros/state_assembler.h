//
// Created by giuseppe on 23.01.21.
//

#pragma once
#include <franka/robot_state.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/Twist.h>

#include <Eigen/Core>

#include "mppi_manipulation/dimensions.h"

namespace manipulation {

/// The scope of this class is to assemble the state vector from multiple source of information

class StateAssembler {
 public:
  StateAssembler(const ros::NodeHandle& nh, bool fixed_base);
  ~StateAssembler() = default;

 public:
  bool get_state(Eigen::VectorXd& x, const franka::RobotState& arm_state);
  std::string state_as_string(const Eigen::VectorXd& x);

  // for debug only
  void publish_ros(const Eigen::VectorXd& x);

 private:
  bool update_base_pose();
  bool update_base_twist();
  void base_twist_callback(const geometry_msgs::TwistConstPtr& msg);
  bool update_arm_state(const franka::RobotState& arm_state);
  bool update_articulation_state();

 private:
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

  // articulation
  double previous_time_;
  bool articulation_first_computation_;
  double theta_;
  double dtheta_;
  geometry_msgs::TransformStamped tf_handle_start_;
  geometry_msgs::TransformStamped tf_handle_current_;

  // debug ros publishing: do not use for realtime control loops
  ros::Publisher base_pose_publisher_;
  ros::Publisher arm_state_publisher_;
  ros::Publisher articulation_state_publisher_;
};
}  // namespace manipulation