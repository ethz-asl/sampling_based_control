//
// Created by giuseppe on 16.08.21.
//

#include "mppi_manipulation_royalpanda/simulation.h"
#include "mppi_manipulation_royalpanda/utils.h"
#include "manipulation_msgs/conversions.h"


using namespace manipulation;
using namespace manipulation_royalpanda;

RoyalPandaSim::RoyalPandaSim(const ros::NodeHandle &nh): nh_(nh) {};

bool RoyalPandaSim::init_sim() {
  if (!init_params()){
    NAMED_LOG_ERROR("Failed to initialize parameters.");
    return false;
  }

  if (!init_dynamics()){
    NAMED_LOG_ERROR("Failed to initialize dynamics.");
    return false;
  }

  init_handles();
  init_publishers();
  return true;
}

bool RoyalPandaSim::init_params() {
  if (!nh_.param<std::vector<std::string>>("arm_joint_names", arm_joint_name_,
                                           {}) ||
      arm_joint_name_.size() != 9) {
    NAMED_LOG_ERROR(
        "Failed to parse arm_joint_names (must contain gripper joints)");
    return false;
  }

  if(!nh_.param<std::string>("base_odom_topic", base_odom_topic_, {})){
    NAMED_LOG_ERROR("Failed to get base_odom_topic");
    return false;
  }

  if(!nh_.param<std::string>("base_twist_topic", base_twist_topic_, {})){
    NAMED_LOG_ERROR("Failed to get base_twist_topic");
    return false;
  }

  if (!nh_.param<std::string>("base_twist_cmd_topic", base_twist_cmd_topic_,
                              {})) {
    NAMED_LOG_ERROR("Failed to get base_twist_cmd_topic");
    return false;
  }

  if(!nh_.param<std::string>("handle_odom_topic", handle_odom_topic_, {})){
    NAMED_LOG_ERROR("Failed to get handle_odom_topic");
    return false;
  }

  if(!nh_.param<std::string>("arm_state_topic", arm_state_topic_, {})){
    NAMED_LOG_ERROR("Failed to get arm_state_topic");
    return false;
  }

  arm_state_.name = arm_joint_name_;
  arm_state_.position.resize(9);
  arm_state_.velocity.resize(9);
  arm_state_.effort.resize(9);
  return true;
}

bool RoyalPandaSim::init_dynamics() {
  bool is_sim = true;
  DynamicsParams params;
  if (!params.init_from_ros(nh_, is_sim)){
    return false;
  }
  ROS_INFO_STREAM(params);
  dynamics_ = std::make_unique<ManipulatorDynamicsRos>(nh_, params);
  return true;
}

void RoyalPandaSim::init_handles() {
  arm_joint_position_.setZero(9);
  arm_joint_velocity_.setZero(9);
  arm_joint_effort_.setZero(9);
  arm_joint_effort_desired_.setZero(9);

  // usual interfaces
  for (int i = 0; i < 9; i++) {
    joint_state_if_.registerHandle(hardware_interface::JointStateHandle(
        arm_joint_name_[i], &arm_joint_position_[i], &arm_joint_velocity_[i],
        &arm_joint_effort_[i]));

    effort_joint_if_.registerHandle(hardware_interface::JointHandle(
        joint_state_if_.getHandle(arm_joint_name_[i]), &arm_joint_effort_desired_[i]));
  }

  // franka interfaces
  franka_state_if_.registerHandle(franka_hw::FrankaStateHandle("panda_robot", this->robot_state_));
  franka_model_if_.registerHandle(
      franka_hw::FrankaModelHandle("panda_model", *this->model_, this->robot_state_));

  registerInterface(&joint_state_if_);
  registerInterface(&effort_joint_if_);
  registerInterface(&franka_state_if_);
  registerInterface(&franka_model_if_);
}

void RoyalPandaSim::init_publishers() {
  base_pose_publisher_ = nh_.advertise<nav_msgs::Odometry>(base_odom_topic_, 1);
  base_twist_publisher_ = nh_.advertise<nav_msgs::Odometry>(base_twist_topic_, 1);
  object_pose_publisher_ = nh_.advertise<nav_msgs::Odometry>(handle_odom_topic_, 1);
  arm_state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>(arm_state_topic_, 1);
  base_twist_cmd_subscriber_ = nh_.subscribe(
      base_twist_cmd_topic_, 1, &RoyalPandaSim::base_twist_cmd_callback, this);
}

void RoyalPandaSim::read_sim(ros::Time time, ros::Duration period) {
  ROS_INFO_STREAM("Current simulation state ("
                  << time.toSec()
                  << ") is: " << dynamics_->get_state().transpose());
  manipulation::conversions::fromEigenState(base_position_,
                                    base_twist_,
                                    arm_joint_position_,
                                    arm_joint_velocity_,
                                    object_position_,
                                    object_velocity_,
                                    contact_state_,
                                    tank_state_,
                                    arm_joint_effort_,
                                    dynamics_->get_state());
  for (int i = 0; i < 7; i++) {
    std::string name =  "panda_joint" + std::to_string(i + 1);
    this->robot_state_.q[i] = arm_joint_position_[i];
    this->robot_state_.dq[i] = arm_joint_velocity_[i];
    this->robot_state_.tau_J[i] = arm_joint_effort_[i];
    this->robot_state_.dtau_J[i] = 0.0;

    this->robot_state_.q_d[i] = arm_joint_position_[i];
    this->robot_state_.dq_d[i] = arm_joint_velocity_[i];
    this->robot_state_.ddq_d[i] = 0.0;
    this->robot_state_.tau_J_d[i] = arm_joint_effort_desired_[i];

    // For now we assume no flexible joints
    this->robot_state_.theta[i] = arm_joint_position_[i];
    this->robot_state_.dtheta[i] = arm_joint_velocity_[i];
    this->robot_state_.tau_ext_hat_filtered[i] = arm_joint_effort_[i];
  }

  this->robot_state_.control_command_success_rate = 1.0;
  this->robot_state_.time = franka::Duration(time.toNSec() / 1e6 /*ms*/);

  // Next try publishing and running the observer node and see what I get
  base_pose_odom_.header.frame_id = "world";
  base_pose_odom_.header.stamp = time;
  base_pose_odom_.pose.pose.position.x = base_position_.x();
  base_pose_odom_.pose.pose.position.y = base_position_.y();
  base_pose_odom_.pose.pose.position.z = base_position_.z();
  base_pose_odom_.pose.pose.orientation.x = 0.0;
  base_pose_odom_.pose.pose.orientation.y = 0.0;
  base_pose_odom_.pose.pose.orientation.z = std::sin(base_position_.z()/2.0);
  base_pose_odom_.pose.pose.orientation.w = std::cos(base_position_.z()/2.0);
  base_pose_publisher_.publish(base_pose_odom_);

  base_twist_odom_.header.frame_id = "world";
  base_twist_odom_.header.stamp = time;
  base_twist_odom_.twist.twist.linear.x = base_twist_.x();
  base_twist_odom_.twist.twist.linear.y = base_twist_.y();
  base_twist_odom_.twist.twist.angular.z = base_twist_.z();
  base_twist_publisher_.publish(base_twist_odom_);

  arm_state_.header.stamp = time;
  for (int i = 0; i < 9; i++) {
    arm_state_.position[i] = arm_joint_position_[i];
    arm_state_.velocity[i] = arm_joint_velocity_[i];
    arm_state_.effort[i] = arm_joint_effort_[i];
  }
  arm_state_publisher_.publish(arm_state_);

  Eigen::Vector3d handle_position;
  Eigen::Quaterniond handle_orientation;
  dynamics_->get_handle_pose(handle_position, handle_orientation);
  handle_odom_.header.frame_id = "world";
  handle_odom_.header.stamp = time;
  handle_odom_.pose.pose.position.x = handle_position.x();
  handle_odom_.pose.pose.position.y = handle_position.y();
  handle_odom_.pose.pose.position.z = handle_position.z();
  handle_odom_.pose.pose.orientation.x = handle_orientation.x();
  handle_odom_.pose.pose.orientation.y = handle_orientation.y();
  handle_odom_.pose.pose.orientation.z = handle_orientation.z();
  handle_odom_.pose.pose.orientation.w = handle_orientation.w();
  object_pose_publisher_.publish(handle_odom_);
}

void RoyalPandaSim::base_twist_cmd_callback(
    const geometry_msgs::TwistConstPtr &msg) {
  base_twist_cmd_.x() = msg->linear.x;
  base_twist_cmd_.y() = msg->linear.y;
  base_twist_cmd_.z() = msg->angular.z;
}

void RoyalPandaSim::publish_ros(){
  dynamics_->publish_ros();
}

void RoyalPandaSim::write_sim(ros::Time time, ros::Duration period){
  // TODO subscriber to the base desired twist

  // base is velocity controlled
  u_.head<3>() = base_twist_cmd_;
  dynamics_->set_control(u_);

  // arm has imperfect gravity compensation
  Eigen::VectorXd tau = 0.9 * dynamics_->get_panda()->getNonlinearities().e();

  // control only the arm joints (leave out gripper)
  tau.segment<7>(3) += arm_joint_effort_desired_.head<7>();
  dynamics_->get_panda()->setGeneralizedForce(tau);
  dynamics_->advance();
}

double RoyalPandaSim::get_time_step() {
  return dynamics_->get_dt();
}