/*!
 * @file     controller_interface.cpp
 * @author   Matthias Studiger
 * @date     19.03.2021
 * @version  1.0
 * @brief    description
 */

#include "mppi_omav_raisim/controller_interface.h"
#include "mppi_omav_raisim/cost.h"
#include "mppi_omav_raisim/dynamics.h"

#include <memory>
#include <ros/package.h>
#include <string>

using namespace omav_raisim;

bool OMAVControllerInterface::init_ros() {
  optimal_rollout_publisher_ =
      nh_.advertise<geometry_msgs::PoseArray>("/optimal_rollout", 10);
  trajectory_publisher_ =
      nh_.advertise<geometry_msgs::PoseArray>("/trajectory", 1000);
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
  if (!nh_.param<std::string>("/robot_description_raisim",
                              robot_description_raisim, "")) {
    throw std::runtime_error(
        "Could not parse robot_description_raisim. Is the parameter set?");
  }

  dynamics = std::make_shared<OMAVRaisimDynamics>(robot_description_raisim,
                                                  config_.step_size);

  std::cout << "Done." << std::endl;

  // -------------------------------
  // cost
  // -------------------------------
  OMAVRaisimCostParam cost_param;
  if (!cost_param.parse_from_ros(nh_)) {
    ROS_ERROR("Failed to parse cost parameters.");
    return false;
  }
  ROS_INFO_STREAM("Successfully parsed cost params: \n" << cost_param);
  auto cost =
      std::make_shared<OMAVRaisimCost>(robot_description_raisim, cost_param);

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

  ref_.rr.resize(1, mppi::observation_t::Zero(7));
  ref_.rr[0](0) = x_goal_position;
  ref_.rr[0](1) = y_goal_position;
  ref_.rr[0](2) = z_goal_position;
  ref_.rr[0](3) = w_goal_quaternion;
  ref_.rr[0](4) = x_goal_quaternion;
  ref_.rr[0](5) = y_goal_quaternion;
  ref_.rr[0](6) = z_goal_quaternion;
  ref_.tt.resize(1, 0.0);

  ROS_INFO_STREAM("Reference initialized with: " << ref_.rr[0].transpose());
  return true;
}

bool OMAVControllerInterface::update_reference() {
  if (!reference_set_)
    get_controller()->set_reference_trajectory(ref_);
  reference_set_ = true;
  return true;
}

void OMAVControllerInterface::publish_ros() {}

void OMAVControllerInterface::publish_optimal_rollout() {
  get_controller()->get_optimal_rollout(optimal_rollout_states_,
                                        optimal_rollout_inputs_);
  geometry_msgs::PoseArray optimal_rollout_array_;
  optimal_rollout_array_.header.frame_id = "odom";
  optimal_rollout_array_.header.stamp = ros::Time::now();
  for (int i = 0; i < optimal_rollout_states_.size(); i++) {
    if ((i % 10) == 0) {
      current_pose_.position.x = optimal_rollout_states_[i](16);
      current_pose_.position.y = optimal_rollout_states_[i](17);
      current_pose_.position.z = optimal_rollout_states_[i](18);
      current_pose_.orientation.w = optimal_rollout_states_[i](9);
      current_pose_.orientation.x = optimal_rollout_states_[i](10);
      current_pose_.orientation.y = optimal_rollout_states_[i](11);
      current_pose_.orientation.z = optimal_rollout_states_[i](12);
      optimal_rollout_array_.poses.push_back(current_pose_);
    }
  }
  optimal_rollout_publisher_.publish(optimal_rollout_array_);
}

void OMAVControllerInterface::publish_trajectories() {
  get_controller()->get_rollout_trajectories(current_trajectories);
  geometry_msgs::PoseArray trajectory_array;
  trajectory_array.header.frame_id = "odom";
  trajectory_array.header.stamp = ros::Time::now();
  for (int i = 0; i < current_trajectories.size(); i++) {
    xx_current_trajectory = current_trajectories[i].xx;
    for (int j = 0; j < xx_current_trajectory.size(); j++) {
      if ((j % 10) == 0) {
        current_trajectory_pose.position.x = xx_current_trajectory[j](16);
        current_trajectory_pose.position.y = xx_current_trajectory[j](17);
        current_trajectory_pose.position.z = xx_current_trajectory[j](18);
        current_trajectory_pose.orientation.w = xx_current_trajectory[j](9);
        current_trajectory_pose.orientation.x = xx_current_trajectory[j](10);
        current_trajectory_pose.orientation.y = xx_current_trajectory[j](11);
        current_trajectory_pose.orientation.z = xx_current_trajectory[j](12);
        trajectory_array.poses.push_back(current_trajectory_pose);
      }
    }
  }
  trajectory_publisher_.publish(trajectory_array);
}
