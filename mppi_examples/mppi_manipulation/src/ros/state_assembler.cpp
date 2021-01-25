//
// Created by giuseppe on 25.01.21.
//

#include "mppi_manipulation/ros/state_assembler.h"

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Geometry>

namespace manipulation {

StateAssembler::StateAssembler(const ros::NodeHandle& nh, bool fixed_base)
    : fixed_base_(fixed_base), nh_(nh), tf2_listener_(tf2_buffer_) {
  std::string base_twist_topic;
  nh_.param<std::string>("base_twist_topic", base_twist_topic, "");

  ros::SubscribeOptions so;
  so.init<geometry_msgs::Twist>(base_twist_topic, 1,
                                boost::bind(&StateAssembler::base_twist_callback, this, _1));
  base_twist_queue_ = std::make_unique<ros::CallbackQueue>();
  so.callback_queue = base_twist_queue_.get();
  so.transport_hints = ros::TransportHints().unreliable().tcpNoDelay();
  base_twist_subscriber_ = nh_.subscribe(so);
  base_twist_received_ = false;
  ROS_INFO_STREAM("Subscribing to base twist at topic: " << base_twist_subscriber_.getTopic());

  q_.setZero();
  dq_.setZero();
  q_.tail<2>() << 0.04, 0.04;  // TODO(giuseppe) hard coded gripper position

  theta_ = 0.0;
  dtheta_ = 0.0;
  articulation_first_computation_ = true;

  // ros debug publishing
  base_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/state_assembler/base_pose", 1);
  arm_state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("/state_assembler/arm_joint_state", 1);
  articulation_state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("/state_assembler/object_joint_state", 1);
}

bool StateAssembler::get_state(Eigen::VectorXd& x, const franka::RobotState& arm_state) {
  bool success = true;
  if (!fixed_base_) {
    success &= update_base_pose();
    success &= update_base_twist();
  }

  success &= update_arm_state(arm_state);
  success &= update_articulation_state();

  if (!success) return success;

  if (!fixed_base_) {
    x = Eigen::VectorXd::Zero(27);  // last is for contact state - unused
    x.head<3>() = base_state_;
    x.segment<9>(3) = q_;
    x.segment<3>(12) = base_twist_;
    x.segment<9>(15) = dq_;
    x(24) = theta_;
    x(25) = dtheta_;
  } else {
    x = Eigen::VectorXd::Zero(21);  // last is for contact state - unused
    x.head<9>() = q_;
    x.tail<9>(9) = dq_;
    x(18) = theta_;
    x(19) = dtheta_;
  }
  return true;
}

std::string StateAssembler::state_as_string(const Eigen::VectorXd& x) {
  std::stringstream ss;
  ss << "\n";
  ss << "=========================================================\n";
  ss << "                        State                            \n";
  ss << "=========================================================\n";
  ss << "Base position:  " << base_state_.transpose() << std::endl;
  ss << "Base twist:     " << base_twist_.transpose() << std::endl;
  ss << "Joint position: " << q_.transpose() << std::endl;
  ss << "Joint velocity: " << dq_.transpose() << std::endl;
  ss << "Theta:          " << theta_ << std::endl;
  ss << "Theta dot:      " << dtheta_ << std::endl;
  ss << "x: " << std::setprecision(2) << x.transpose() << std::endl;
  return ss.str();
}

bool StateAssembler::update_base_pose() {
  try {
    // get the latest transform
    tf_base_ = tf2_buffer_.lookupTransform("world", "base", ros::Time(0));
  } catch (tf2::TransformException& ex) {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }

  base_state_.x() = tf_base_.transform.translation.x;
  base_state_.y() = tf_base_.transform.translation.y;

  // set to zero pitch and roll components and normalize.
  Eigen::Matrix3d m(Eigen::Quaterniond(tf_base_.transform.rotation.w, tf_base_.transform.rotation.x,
                                       tf_base_.transform.rotation.y,
                                       tf_base_.transform.rotation.z));

  Eigen::Vector3d world_ix = m.col(0);  // 2d projection of forward axis
  base_state_.z() = std::atan2(world_ix.y(), world_ix.x());
  return true;
}

bool StateAssembler::update_base_twist() {
  base_twist_queue_->callAvailable();
  if (!base_twist_received_) {
    ROS_ERROR("Failed to update base twist");
    return false;
  }
  return true;
}

void StateAssembler::base_twist_callback(const geometry_msgs::TwistConstPtr& msg) {
  std::unique_lock<std::mutex> lock(base_twist_mutex_);
  base_twist_.x() = msg->linear.x;
  base_twist_.y() = msg->linear.y;
  base_twist_.z() = msg->angular.z;
  base_twist_received_ = true;
}

bool StateAssembler::update_arm_state(const franka::RobotState& arm_state) {
  for (size_t i = 0; i < 7; i++) q_(i) = arm_state.q[i];
  for (size_t i = 0; i < 7; i++) dq_(i) = arm_state.dq[i];
  return true;
}

bool StateAssembler::update_articulation_state() {
  try {
    // get the latest transform
    tf_handle_current_ = tf2_buffer_.lookupTransform("door_hinge", "handle", ros::Time(0));
  } catch (tf2::TransformException& ex) {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }

  if (articulation_first_computation_) {
    tf_handle_start_ = tf_handle_current_;
    articulation_first_computation_ = false;
    previous_time_ = ros::Time::now().toSec();
    return true;
  }

  double abs_theta_current = std::atan2(tf_handle_current_.transform.translation.x,
                                        tf_handle_current_.transform.translation.y);
  double abs_theta_start = std::atan2(tf_handle_start_.transform.translation.x,
                                      tf_handle_start_.transform.translation.y);

  double theta_new = abs_theta_current - abs_theta_start;

  double current_time = ros::Time::now().toSec();
  dtheta_ = (theta_new - theta_) / (current_time - previous_time_);
  theta_ = theta_new;
  previous_time_ = current_time;

  return true;
}

void StateAssembler::publish_ros(const Eigen::VectorXd& x) {
  if (!fixed_base_) {
    geometry_msgs::PoseStamped base_pose;
    base_pose.header.stamp = ros::Time::now();
    base_pose.header.frame_id = "world";
    base_pose.pose.position.x = x(0);
    base_pose.pose.position.y = x(1);
    base_pose.pose.position.z = 0.0;
    Eigen::Quaterniond q(Eigen::AngleAxisd(x(2), Eigen::Vector3d::UnitZ()));
    base_pose.pose.orientation.x = q.x();
    base_pose.pose.orientation.y = q.y();
    base_pose.pose.orientation.z = q.z();
    base_pose.pose.orientation.w = q.w();
    base_pose_publisher_.publish(base_pose);
  }
}
}  // namespace manipulation