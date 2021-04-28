/*!
 * @file     controller_interface.cpp
 * @author   Matthias Studiger
 * @date     10.04.2021
 * @version  1.0
 * @brief    description
 */

#include "mppi_omav_interaction/controller_interface.h"
#include "mppi_omav_interaction/cost.h"
#include "mppi_omav_interaction/dynamics.h"

#include <memory>
#include <ros/package.h>
#include <string>

using namespace omav_interaction;

bool OMAVControllerInterface::init_ros() {
  optimal_rollout_publisher_ =
      nh_.advertise<geometry_msgs::PoseArray>("/optimal_rollout", 1);
  trajectory_publisher_ =
      nh_.advertise<geometry_msgs::PoseArray>("/trajectory", 1);
  reference_subscriber_ =
      nh_.subscribe("/mppi_pose_desired", 10,
                    &OMAVControllerInterface::desired_pose_callback, this);
  mode_subscriber_ = nh_.subscribe(
      "/mppi_omav_mode", 10, &OMAVControllerInterface::mode_callback, this);
  object_reference_subscriber_ =
      nh_.subscribe("/mppi_object_desired", 10,
                    &OMAVControllerInterface::object_reference_callback, this);

  cmd_multi_dof_joint_trajectory_pub_ =
      nh_public_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
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
  std::string robot_description_raisim;
  std::string object_description_raisim;
  if (!nh_.param<std::string>("/robot_description_raisim",
                              robot_description_raisim, "")) {
    throw std::runtime_error(
        "Could not parse robot_description_raisim. Is the parameter set?");
  }
  if (!nh_.param<std::string>("/object_description_raisim",
                              object_description_raisim, "")) {
    throw std::runtime_error(
        "Could not parse object_description_raisim. Is the parameter set?");
  }

  dynamics = std::make_shared<OMAVVelocityDynamics>(
      robot_description_raisim, object_description_raisim, config_.step_size);

  std::cout << "Done." << std::endl;

  // -------------------------------
  // cost
  // -------------------------------
  OMAVInteractionCostParam cost_param;
  if (!cost_param.parse_from_ros(nh_)) {
    ROS_ERROR("Failed to parse cost parameters.");
    return false;
  }
  ROS_INFO_STREAM("Successfully parsed cost params: \n" << cost_param);
  auto cost = std::make_shared<OMAVInteractionCost>(robot_description_raisim,
                                                    cost_param);

  // -------------------------------
  // controller
  // -------------------------------
  controller = std::make_shared<mppi::PathIntegral>(dynamics, cost, config_);

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

  ref_.rr.resize(1, mppi::observation_t::Zero(15));
  ref_.rr[0](0) = x_goal_position;
  ref_.rr[0](1) = y_goal_position;
  ref_.rr[0](2) = z_goal_position;
  ref_.rr[0](3) = w_goal_quaternion;
  ref_.rr[0](4) = x_goal_quaternion;
  ref_.rr[0](5) = y_goal_quaternion;
  ref_.rr[0](6) = z_goal_quaternion;
  ref_.rr[0](7) = 2.5;
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
  get_controller()->set_reference_trajectory(ref_);
}

void OMAVControllerInterface::mode_callback(
    const std_msgs::Int64ConstPtr &msg) {
  std::unique_lock<std::mutex> lock(reference_mutex_);
  if (msg->data == 2.0) {
    x_0_temp = get_controller()->get_current_observation();
    ref_.rr[0](0) = x_0_temp(0) + 0.13;
    std::cout << x_0_temp(0) << std::endl;
    ref_.rr[0](1) = x_0_temp(1);
    ref_.rr[0](2) = x_0_temp(2);
    ref_.rr[0](3) = 1.0;
    ref_.rr[0](4) = 0.0;
    ref_.rr[0](5) = 0.0;
    ref_.rr[0](6) = 0.0;
    ref_.rr[0](8) = 0.0;
  } else {
    ref_.rr[0](8) = msg->data;
  }
  get_controller()->set_reference_trajectory(ref_);
  ROS_INFO_STREAM("Switching to mode:" << msg->data);
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

bool OMAVControllerInterface::set_reference(const observation_t &x) {
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

bool OMAVControllerInterface::set_reset_reference(const observation_t &x) {
  ref_.rr[0](0) = x(0) - 0.1;
  ref_.rr[0](1) = x(1);
  ref_.rr[0](2) = x(2);
  ref_.rr[0](3) = 1;
  ref_.rr[0](4) = 0;
  ref_.rr[0](5) = 0;
  ref_.rr[0](6) = x(6);
  ref_.rr[0](14) = 0;
  get_controller()->set_reference_trajectory(ref_);
  return true;
}

bool OMAVControllerInterface::set_object_reference() {
  ref_.rr[0](7) = 4;
  ref_.rr[0](8) = 0;
  ref_.rr[0](9) = 0.15;
  ref_.rr[0](10) = 1;
  ref_.rr[0](11) = 0;
  ref_.rr[0](12) = 0;
  ref_.rr[0](13) = 0;
  ref_.rr[0](14) = 1;
  get_controller()->set_reference_trajectory(ref_);
  return true;
}

void OMAVControllerInterface::publish_ros() {
  get_controller()->get_optimal_rollout(xx_opt, uu_opt);
  OMAVControllerInterface::publish_trajectory(xx_opt);
}

void OMAVControllerInterface::publish_trajectory(
    const mppi::observation_array_t &x_opt) {
  omav_interaction::conversions::to_trajectory_msg(x_opt,
                                                   current_trajectory_msg_);
  current_trajectory_msg_.header.stamp = ros::Time::now();
  cmd_multi_dof_joint_trajectory_pub_.publish(current_trajectory_msg_);
}

void OMAVControllerInterface::publish_trajectories() {
  get_controller()->get_rollout_trajectories(current_trajectories);
  geometry_msgs::PoseArray trajectory_array;
  geometry_msgs::Pose current_trajectory_pose;
  trajectory_array.header.frame_id = "odom";
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
  geometry_msgs::PoseArray optimal_rollout_array_;
  geometry_msgs::Pose current_pose_;
  optimal_rollout_array_.header.frame_id = "odom";
  optimal_rollout_array_.header.stamp = ros::Time::now();
  for (int i = 0; i < 30; i++) {
    omav_interaction::conversions::PoseMsgFromVector(optimal_rollout_states_[i],
                                                     current_pose_);
    optimal_rollout_array_.poses.push_back(current_pose_);
  }
  optimal_rollout_publisher_.publish(optimal_rollout_array_);
}
