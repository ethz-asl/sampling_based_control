//
// Created by giuseppe on 16.08.21.
//

#include "mppi_manipulation_royalpanda/simulation.h"
#include "manipulation_msgs/conversions.h"
#include "mppi_manipulation_royalpanda/utils.h"

using namespace manipulation;
using namespace manipulation_royalpanda;

RoyalPandaSim::RoyalPandaSim(const ros::NodeHandle &nh) : nh_(nh){};

bool RoyalPandaSim::init_sim() {
  if (!init_params()) {
    ROS_ERROR("Failed to initialize parameters.");
    return false;
  }

  if (!init_dynamics()) {
    ROS_ERROR("Failed to initialize dynamics.");
    return false;
  }

  init_handles();
  init_publishers();
  return true;
}

bool RoyalPandaSim::init_params() {
  if (!nh_.param<std::vector<std::string>>("arm_joint_names", arm_joint_name_,
                                           {}) ||
      arm_joint_name_.size() != 7) {
    ROS_ERROR("Failed to parse arm_joint_names");
    return false;
  }

  if (!nh_.param<std::vector<std::string>>("finger_joint_names",
                                           finger_joint_name_, {}) ||
      finger_joint_name_.size() != 2) {
    ROS_ERROR("Failed to parse finger_joint_names");
    return false;
  }

  if (!nh_.param<std::vector<std::string>>("base_joint_names", base_joint_name_,
                                           {}) ||
      base_joint_name_.size() != 3) {
    ROS_ERROR("Failed to parse base_joint_names");
    return false;
  }

  if (!nh_.param<std::string>("base_odom_topic", base_odom_topic_, {})) {
    ROS_ERROR("Failed to get base_odom_topic");
    return false;
  }

  if (!nh_.param<std::string>("base_twist_topic", base_twist_topic_, {})) {
    ROS_ERROR("Failed to get base_twist_topic");
    return false;
  }

  if (!nh_.param<std::string>("base_twist_cmd_topic", base_twist_cmd_topic_,
                              {})) {
    ROS_ERROR("Failed to get base_twist_cmd_topic");
    return false;
  }

  if (!nh_.param<std::string>("handle_odom_topic", handle_odom_topic_, {})) {
    ROS_ERROR("Failed to get handle_odom_topic");
    return false;
  }

  if (!nh_.param<std::string>("arm_state_topic", arm_state_topic_, {})) {
    ROS_ERROR("Failed to get arm_state_topic");
    return false;
  }

  if (!nh_.param<std::string>("finger_state_topic", finger_state_topic_, {})) {
    ROS_ERROR("Failed to get finger_state_topic");
    return false;
  }

  if (!nh_.param<std::string>("wrench_topic", wrench_topic_, {})) {
    ROS_ERROR("Failed to get wrench_topic");
    return false;
  }

  arm_state_.name = arm_joint_name_;
  arm_state_.position.resize(7);
  arm_state_.velocity.resize(7);
  arm_state_.effort.resize(7);

  finger_state_.name = finger_joint_name_;
  finger_state_.position.resize(2);
  finger_state_.velocity.resize(2);
  finger_state_.effort.resize(2);

  tau_ext_base_arm_.setZero(12);
  return true;
}

bool RoyalPandaSim::init_dynamics() {
  bool is_sim = true;
  DynamicsParams params;
  if (!params.init_from_ros(nh_, is_sim)) {
    return false;
  }
  ROS_INFO_STREAM(params);
  dynamics_ = std::make_unique<ManipulatorDynamicsRos>(nh_, params);
  return true;
}

void RoyalPandaSim::init_handles() {
  // the simulation "arm state" contains also the finger state
  arm_joint_position_.setZero(9);
  arm_joint_velocity_.setZero(9);
  arm_joint_effort_.setZero(9);
  arm_joint_effort_desired_.setZero(9);

  // usual interfaces
  for (int i = 0; i < 7; i++) {
    joint_state_if_.registerHandle(hardware_interface::JointStateHandle(
        arm_joint_name_[i], &arm_joint_position_[i], &arm_joint_velocity_[i],
        &arm_joint_effort_[i]));

    effort_joint_if_.registerHandle(hardware_interface::JointHandle(
        joint_state_if_.getHandle(arm_joint_name_[i]),
        &arm_joint_effort_desired_[i]));
  }

  for (int i = 0; i < 3; i++) {
    joint_state_if_.registerHandle(hardware_interface::JointStateHandle(
        base_joint_name_[i], &base_position_[i], &base_twist_[i],
        &base_effort_[i]));

    velocity_joint_if_.registerHandle(hardware_interface::JointHandle(
        joint_state_if_.getHandle(base_joint_name_[i]),
        &base_twist_cmd_shared_[i]));
  }

  // franka interfaces
  franka_state_if_.registerHandle(
      franka_hw::FrankaStateHandle("panda_robot", this->robot_state_));
  franka_model_if_.registerHandle(franka_hw::FrankaModelHandle(
      "panda_model", *this->model_, this->robot_state_));

  registerInterface(&joint_state_if_);
  registerInterface(&velocity_joint_if_);
  registerInterface(&effort_joint_if_);
  registerInterface(&franka_state_if_);
  registerInterface(&franka_model_if_);
}

void RoyalPandaSim::init_publishers() {
  base_pose_publisher_ = nh_.advertise<nav_msgs::Odometry>(base_odom_topic_, 1);
  base_twist_publisher_ =
      nh_.advertise<nav_msgs::Odometry>(base_twist_topic_, 1);
  object_pose_publisher_ =
      nh_.advertise<nav_msgs::Odometry>(handle_odom_topic_, 1);
  arm_state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>(arm_state_topic_, 1);
  finger_state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>(finger_state_topic_, 1);
  wrench_publisher_ =
      nh_.advertise<geometry_msgs::WrenchStamped>(wrench_topic_, 1);
  base_twist_cmd_subscriber_ = nh_.subscribe(
      base_twist_cmd_topic_, 1, &RoyalPandaSim::base_twist_cmd_callback, this);
  external_force_subscriber_ = nh_.subscribe(
      "/apply_external_force", 1, &RoyalPandaSim::apply_external_force_callback, this);
}

void RoyalPandaSim::print_state() {
  std::cout << "sim state = [" << std::setprecision(2) << dynamics_->get_state().transpose() << std::endl;
}

Eigen::VectorXd RoyalPandaSim::get_state() const {
  return dynamics_->get_state();
}

void RoyalPandaSim::read_sim(ros::Time time, ros::Duration period) {
  // clang-format off
  conversions::fromEigenState(base_position_,
                              base_twist_,
                              base_effort_,
                              arm_joint_position_,
                              arm_joint_velocity_,
                              arm_joint_effort_,
                              object_position_,
                              object_velocity_,
                              contact_state_,
                              tank_state_,
                              dynamics_->get_state());
  // clang-format on
  for (int i = 0; i < 7; i++) {
    std::string name = "panda_joint" + std::to_string(i + 1);
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

  // The base odometry (using vicon) is actually the odometry measurements
  // of a different reference frame attached to the base
  Eigen::Vector3d ref_position;
  Eigen::Quaterniond ref_orientation;
  dynamics_->get_reference_link_pose(ref_position, ref_orientation);
  base_pose_odom_.header.frame_id = "world";
  base_pose_odom_.header.stamp = time;
  base_pose_odom_.pose.pose.position.x = ref_position.x();
  base_pose_odom_.pose.pose.position.y = ref_position.y();
  base_pose_odom_.pose.pose.position.z = ref_position.z();
  base_pose_odom_.pose.pose.orientation.x = ref_orientation.x();
  base_pose_odom_.pose.pose.orientation.y = ref_orientation.y();
  base_pose_odom_.pose.pose.orientation.z = ref_orientation.z();
  base_pose_odom_.pose.pose.orientation.w = ref_orientation.w();
  base_pose_publisher_.publish(base_pose_odom_);

  base_twist_odom_.header.frame_id = "world";
  base_twist_odom_.header.stamp = time;
  base_twist_odom_.twist.twist.linear.x = base_twist_.x();
  base_twist_odom_.twist.twist.linear.y = base_twist_.y();
  base_twist_odom_.twist.twist.angular.z = base_twist_.z();
  base_twist_publisher_.publish(base_twist_odom_);

  arm_state_.header.stamp = time;
  for (int i = 0; i < 7; i++) {
    arm_state_.position[i] = arm_joint_position_[i];
    arm_state_.velocity[i] = arm_joint_velocity_[i];
    arm_state_.effort[i] = arm_joint_effort_[i];
  }
  arm_state_publisher_.publish(arm_state_);

  // fingers (like on hardware) treated separately
  finger_state_.header.stamp = time;
  for (int i=0; i<2; i++){
    finger_state_.position[i] = arm_joint_position_[7+i];
    finger_state_.velocity[i] = arm_joint_velocity_[7+i];
    finger_state_.effort[i] = arm_joint_effort_[7+i];
  }
  finger_state_publisher_.publish(finger_state_);

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

  dynamics_->get_ee_jacobian(ee_jacobian_);
  pseudoInverse(ee_jacobian_.transpose(), ee_jacobian_transpose_pinv_);
  tau_ext_base_arm_.head<3>() = base_effort_;
  tau_ext_base_arm_.tail<9>() = arm_joint_effort_;
  ee_wrench_ = ee_jacobian_transpose_pinv_ * tau_ext_base_arm_;

  // this wrench is indeed in the world frame
  // rotate it back to the end effector frame
  Eigen::Vector3d ee_position;
  Eigen::Quaterniond ee_rotation;
  dynamics_->get_end_effector_pose(ee_position, ee_rotation);
  Eigen::Matrix3d R_world_ee(ee_rotation);
  ee_wrench_.head<3>() = R_world_ee.transpose() * ee_wrench_.head<3>();
  ee_wrench_.tail<3>() = R_world_ee.transpose() * ee_wrench_.tail<3>();

  wrench_.header.stamp = time;
  wrench_.header.frame_id = "panda_hand";
  wrench_.wrench.force.x = ee_wrench_(0);
  wrench_.wrench.force.y = ee_wrench_(1);
  wrench_.wrench.force.z = ee_wrench_(2);
  wrench_.wrench.torque.x = ee_wrench_(3);
  wrench_.wrench.torque.y = ee_wrench_(4);
  wrench_.wrench.torque.z = ee_wrench_(5);
  wrench_publisher_.publish(wrench_);
}

void RoyalPandaSim::base_twist_cmd_callback(
    const geometry_msgs::TwistConstPtr &msg) {
  base_twist_cmd_.x() = msg->linear.x;
  base_twist_cmd_.y() = msg->linear.y;
  base_twist_cmd_.z() = msg->angular.z;
}

void RoyalPandaSim::apply_external_force_callback(const geometry_msgs::WrenchConstPtr &msg) {
  user_force_.x() = msg->force.x;
  user_force_.y() = msg->force.y;
  user_force_.z() = msg->force.z;
  ROS_INFO_STREAM("Setting user force to " << user_force_);
}

void RoyalPandaSim::publish_ros() { dynamics_->publish_ros(); }

void RoyalPandaSim::advance_sim(ros::Time time, ros::Duration period) {
  // base is velocity controlled
  u_.head<3>() = base_twist_cmd_shared_;
  dynamics_->set_control(u_);

  // arm has imperfect gravity compensation
  Eigen::VectorXd tau = 0.99 * dynamics_->get_panda()->getNonlinearities().e();

  // control only the arm joints (leave out gripper)
  tau.segment<7>(3) += arm_joint_effort_desired_.head<7>();
  dynamics_->get_panda()->setGeneralizedForce(tau);
  dynamics_->set_external_ee_force(user_force_);
  dynamics_->advance();
}

double RoyalPandaSim::get_time_step() { return dynamics_->get_dt(); }