/*!
 * @file     controller_interface.cpp
 * @author   Matthias Studiger
 * @date     10.04.2021
 * @version  1.0
 * @brief    description
 */

#include "mppi_omav_interaction/controller_interface.h"

#include <memory>
#include <ros/package.h>
#include <string>

using namespace omav_interaction;

bool OMAVControllerInterface::init_ros() {
  // Initialize publisher
  optimal_rollout_publisher_ =
      nh_.advertise<geometry_msgs::PoseArray>("/optimal_rollout", 1);
  optimal_rollout_des_publisher_ =
      nh_.advertise<geometry_msgs::PoseArray>("/optimal_rollout_desired", 1);
  cmd_multi_dof_joint_trajectory_pub_ =
      nh_public_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
  mppi_reference_publisher_ =
      nh_.advertise<geometry_msgs::Pose>("/mppi_reference", 1);
  object_state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("/object/joint_state", 10);
  detailed_publishing_ = nh_.param<bool>("detailed_publishing", false);
  if (detailed_publishing_) {
    trajectory_publisher_ =
        nh_.advertise<geometry_msgs::PoseArray>("/trajectory", 1);

    cost_publisher_ = nh_.advertise<mppi_ros::Array>("/cost_parts", 10);
    normalized_force_publisher_ =
        nh_.advertise<visualization_msgs::MarkerArray>("/normalized_forces",
                                                       10);
  }

  // Initialize subscriber
  reference_subscriber_ =
      nh_.subscribe("/mppi_pose_desired", 10,
                    &OMAVControllerInterface::desired_pose_callback, this);
  mode_subscriber_ = nh_.subscribe(
      "/mppi_omav_mode", 10, &OMAVControllerInterface::mode_callback, this);
  object_reference_subscriber_ =
      nh_.subscribe("/mppi_object_desired", 10,
                    &OMAVControllerInterface::object_reference_callback, this);

  object_state_.name = {"articulation_joint"};
  object_state_.position.resize(1);
  return true;
}

bool OMAVControllerInterface::set_controller(
    std::shared_ptr<mppi::PathIntegral> &controller) {
  // -------------------------------
  // config
  // -------------------------------
  std::string config_file;
  if (!nh_.param<std::string>("/config_file", config_file, "")) {
    throw std::runtime_error(
        "Could not parse config_file. Is the parameter set?");
  }
  if (!config_.init_from_file(config_file)) {
    ROS_ERROR_STREAM("Failed to init solver options from " << config_file);
    return false;
  }
  // -------------------------------
  // dynamics
  // -------------------------------
  mppi::DynamicsBase::dynamics_ptr dynamics;

  if (!nh_.param<std::string>("/robot_description_raisim",
                              robot_description_raisim_, "")) {
    throw std::runtime_error(
        "Could not parse robot_description_raisim. Is the parameter set?");
  }
  if (!nh_.param<std::string>("/object_description", object_description_, "")) {
    throw std::runtime_error(
        "Could not parse object_description. Is the parameter set?");
  }
  if (!nh_.param<std::string>("/robot_description_pinocchio",
                              robot_description_pinocchio_, "")) {
    throw std::runtime_error(
        "Could not parse robot_description_pinocchio. Is the parameter set?");
  }
  dynamics = std::make_shared<OMAVVelocityDynamics>(
      robot_description_raisim_, object_description_, config_.step_size);

  ROS_INFO_STREAM("Done.");

  // -------------------------------
  // cost
  // -------------------------------

  if (!cost_param_.parse_from_ros(nh_)) {
    ROS_ERROR("Failed to parse cost parameters.");
    return false;
  }
  ROS_INFO_STREAM("Successfully parsed cost params: \n" << cost_param_);
  cost_ = std::make_shared<OMAVInteractionCost>(
      robot_description_raisim_, robot_description_pinocchio_,
      object_description_, &cost_param_);

  // -------------------------------
  // controller
  // -------------------------------
  controller = std::make_shared<mppi::PathIntegral>(dynamics, cost_, config_);

  // -------------------------------
  // initialize reference
  // -------------------------------
  double x_goal_position, y_goal_position, z_goal_position, w_goal_quaternion,
      x_goal_quaternion, y_goal_quaternion, z_goal_quaternion;
  nh_.param<double>("goal_position_x", x_goal_position, 0.0);
  nh_.param<double>("goal_position_y", y_goal_position, 0.0);
  nh_.param<double>("goal_position_z", z_goal_position, 0.0);
  nh_.param<double>("goal_quaternion_w", w_goal_quaternion, 1.0);
  nh_.param<double>("goal_quaternion_x", x_goal_quaternion, 0.0);
  nh_.param<double>("goal_quaternion_y", y_goal_quaternion, 0.0);
  nh_.param<double>("goal_quaternion_z", z_goal_quaternion, 0.0);

  ref_.rr.resize(1, mppi::observation_t::Zero(9));
  ref_.rr[0](0) = x_goal_position;
  ref_.rr[0](1) = y_goal_position;
  ref_.rr[0](2) = z_goal_position;
  ref_.rr[0](3) = w_goal_quaternion;
  ref_.rr[0](4) = x_goal_quaternion;
  ref_.rr[0](5) = y_goal_quaternion;
  ref_.rr[0](6) = z_goal_quaternion;
  ref_.rr[0](7) = 0.0;
  // Initialize mode
  ref_.rr[0](8) = 0;
  ref_.tt.resize(1, 0.0);

  ROS_INFO_STREAM("Reference initialized with: " << ref_.rr[0].transpose());
  return true;
}

void OMAVControllerInterface::desired_pose_callback(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  std::unique_lock<std::mutex> lock(reference_mutex_);
  ref_.rr[0](0) = msg->pose.position.x;
  ref_.rr[0](1) = msg->pose.position.y;
  ref_.rr[0](2) = msg->pose.position.z;
  ref_.rr[0](3) = msg->pose.orientation.w;
  ref_.rr[0](4) = msg->pose.orientation.x;
  ref_.rr[0](5) = msg->pose.orientation.y;
  ref_.rr[0](6) = msg->pose.orientation.z;
  std::cout << "Desired Pose Callback" << std::endl;
  get_controller()->set_reference_trajectory(ref_);
}

void OMAVControllerInterface::mode_callback(
    const std_msgs::Int64ConstPtr &msg) {
  std::unique_lock<std::mutex> lock(reference_mutex_);
  ref_.rr[0](8) = msg->data;
  ROS_INFO_STREAM("Switching to mode:" << msg->data);
  get_controller()->set_reference_trajectory(ref_);
}

void OMAVControllerInterface::object_reference_callback(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  std::unique_lock<std::mutex> lock(reference_mutex_);
  ref_.rr[0](7) = msg->pose.position.x;
  get_controller()->set_reference_trajectory(ref_);
}

bool OMAVControllerInterface::update_reference() {
  if (!reference_set_)
    get_controller()->set_reference_trajectory(ref_);
  reference_set_ = true;
  return true;
}

bool OMAVControllerInterface::update_cost_param(
    const OMAVInteractionCostParam &cost_param) {
  cost_param_ = cost_param;
  return false;
}

bool OMAVControllerInterface::set_initial_reference(const observation_t &x) {
  ref_.rr[0](0) = x(0);
  ref_.rr[0](1) = x(1);
  ref_.rr[0](2) = x(2);
  ref_.rr[0](3) = x(3);
  ref_.rr[0](4) = x(4);
  ref_.rr[0](5) = x(5);
  ref_.rr[0](6) = x(6);
  get_controller()->set_reference_trajectory(ref_);
  return true;
}

void OMAVControllerInterface::publish_ros() {
  get_controller()->get_optimal_rollout(xx_opt, uu_opt);
  OMAVControllerInterface::publish_trajectory(xx_opt, uu_opt);
  OMAVControllerInterface::publish_optimal_rollout();
  static tf::TransformBroadcaster odom_broadcaster;
  tf::Transform omav_odom;
  omav_odom.setOrigin(tf::Vector3(xx_opt[0](0), xx_opt[0](1), xx_opt[0](2)));
  omav_odom.setRotation(
      tf::Quaternion(xx_opt[0](4), xx_opt[0](5), xx_opt[0](6), xx_opt[0](3)));
  odom_broadcaster.sendTransform(
      tf::StampedTransform(omav_odom, ros::Time::now(), "world", "odom_omav"));
  // update object state visualization
  object_state_.header.stamp = ros::Time::now();
  object_state_.position[0] = xx_opt[0](13);
  object_state_publisher_.publish(object_state_);
}

void OMAVControllerInterface::publish_trajectory(
    const mppi::observation_array_t &x_opt, const mppi::input_array_t &u_opt) {
  omav_interaction::conversions::to_trajectory_msg(x_opt, u_opt,
                                                   current_trajectory_msg_);
  current_trajectory_msg_.header.stamp = ros::Time::now();
  cmd_multi_dof_joint_trajectory_pub_.publish(current_trajectory_msg_);
}

void OMAVControllerInterface::publish_all_trajectories() {
  get_controller()->get_rollout_trajectories(current_trajectories);
  geometry_msgs::PoseArray trajectory_array;
  geometry_msgs::Pose current_trajectory_pose;
  trajectory_array.header.frame_id = "world";
  trajectory_array.header.stamp = ros::Time::now();
  for (int i = 0; i < current_trajectories.size(); i++) {
    xx_current_trajectory = current_trajectories[i].xx;
    for (int j = 0; j < xx_current_trajectory.size(); j++) {
      if ((j % 10) == 0) {
        omav_interaction::conversions::PoseMsgFromVector(
            xx_current_trajectory[j], current_trajectory_pose);
        trajectory_array.poses.push_back(current_trajectory_pose);
      }
    }
  }
  trajectory_publisher_.publish(trajectory_array);
}

void OMAVControllerInterface::publish_optimal_rollout() {
  get_controller()->get_optimal_rollout(optimal_rollout_states_,
                                        optimal_rollout_inputs_);
  geometry_msgs::PoseArray optimal_rollout_array, optimal_rollout_array_des;
  geometry_msgs::Pose current_pose, current_pose_des, mppi_reference;
  visualization_msgs::Marker force_marker;

  omav_interaction::conversions::arrow_initialization(force_marker);

  visualization_msgs::MarkerArray force_marker_array;
  Eigen::Vector3d force;
  Eigen::Vector3d force_normalized;
  optimal_rollout_array.header.frame_id = "world";
  optimal_rollout_array.header.stamp = ros::Time::now();
  optimal_rollout_array_des.header.frame_id = "world";
  optimal_rollout_array_des.header.stamp = ros::Time::now();

  velocity_cost_ = 0;
  power_cost_ = 0;
  object_cost_ = 0;
  torque_cost_ = 0;
  handle_hook_cost_ = 0;
  pose_cost_ = 0;
  overall_cost_ = 0;

  for (int i = 0; i < optimal_rollout_states_.size(); i++) {
    omav_interaction::conversions::PoseMsgFromVector(optimal_rollout_states_[i],
                                                     current_pose);
    omav_interaction::conversions::PoseMsgFromVector(
        optimal_rollout_states_[i].segment<15>(19), current_pose_des);
    optimal_rollout_array.poses.push_back(current_pose);
    optimal_rollout_array_des.poses.push_back(current_pose_des);
    if (detailed_publishing_) {
      // Cost publishing
      cost_->compute_cost(optimal_rollout_states_[i], ref_.rr[0], i * 0.015);
      // Velocity Cost
      velocity_cost_ += cost_->tip_velocity_cost_;
      // Efficiency Cost
      power_cost_ += cost_->efficiency_cost_;
      // Object Cost
      object_cost_ += cost_->object_cost_;
      // Torque Cost
      torque_cost_ += cost_->torque_cost_;
      // Handle Hook Cost
      handle_hook_cost_ += cost_->handle_hook_cost_;
      // Pose Cost
      pose_cost_ += cost_->pose_cost_;
      overall_cost_ += cost_->cost_;

      force_normed_ = optimal_rollout_states_[i].segment<3>(15).normalized();

      force_marker.points[0].x = cost_->hook_pos_(0);
      force_marker.points[0].y = cost_->hook_pos_(1);
      force_marker.points[0].z = cost_->hook_pos_(2);
      force_marker.points[1].x = cost_->hook_pos_(0) + force_normed_(0);
      force_marker.points[1].y = cost_->hook_pos_(1) + force_normed_(1);
      force_marker.points[1].z = cost_->hook_pos_(2) + force_normed_(2);
      force_marker.id = i;

      force_marker_array.markers.push_back(force_marker);
    }
  }
  if (detailed_publishing_) {
    mppi_ros::Array cost_array_message_;
    cost_array_message_.array.push_back(velocity_cost_);
    cost_array_message_.array.push_back(power_cost_);
    cost_array_message_.array.push_back(object_cost_);
    cost_array_message_.array.push_back(torque_cost_);
    cost_array_message_.array.push_back(handle_hook_cost_);
    cost_array_message_.array.push_back(optimal_rollout_states_[1](15));
    cost_array_message_.array.push_back(optimal_rollout_states_[1](16));
    cost_array_message_.array.push_back(optimal_rollout_states_[1](17));
    cost_array_message_.array.push_back(
        optimal_rollout_states_[1].segment<3>(15).norm());
    cost_array_message_.array.push_back(
        optimal_rollout_states_[1].segment<3>(15).cross(com_hook_)(0));
    cost_array_message_.array.push_back(
        optimal_rollout_states_[1].segment<3>(15).cross(com_hook_)(1));
    cost_array_message_.array.push_back(
        optimal_rollout_states_[1].segment<3>(15).cross(com_hook_)(2));
    cost_array_message_.array.push_back(
        optimal_rollout_states_[1].segment<3>(15).cross(com_hook_).norm());
    cost_array_message_.array.push_back(
        acos(optimal_rollout_states_[1].segment<3>(15).normalized().dot(
            com_hook_.normalized())) *
        180.0 / M_PI);
    cost_array_message_.array.push_back(pose_cost_);
    cost_array_message_.array.push_back(overall_cost_);
    cost_publisher_.publish(cost_array_message_);
    normalized_force_publisher_.publish(force_marker_array);
  }
  mppi_reference.position.x = ref_.rr[0](0);
  mppi_reference.position.y = ref_.rr[0](1);
  mppi_reference.position.z = ref_.rr[0](2);
  mppi_reference.orientation.w = ref_.rr[0](3);
  mppi_reference.orientation.x = ref_.rr[0](4);
  mppi_reference.orientation.y = ref_.rr[0](5);
  mppi_reference.orientation.z = ref_.rr[0](6);

  optimal_rollout_publisher_.publish(optimal_rollout_array);
  optimal_rollout_des_publisher_.publish(optimal_rollout_array_des);
  mppi_reference_publisher_.publish(mppi_reference);
}

// namespace omav_interaction
