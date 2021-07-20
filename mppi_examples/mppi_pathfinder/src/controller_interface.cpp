/*!
 * @file     controller_ros.cpp
 * @author   Giuseppe Rizzi
 * @date     03.09.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_pathfinder/controller_interface.h"
#include <ros/package.h>

using namespace pathfinder;

bool PathfinderControllerInterface::set_controller(
    std::shared_ptr<mppi::PathIntegral> &controller) {
  // -------------------------------
  // dynamics
  // -------------------------------
  auto dynamics_config = PathfinderDynamicsConfig();
  dynamics_config.mc = nh_.param<double>("dynamics/mass_cart", 1.0);
  dynamics_config.tau_theta = nh_.param<double>("dynamics/tau_theta", 0.7);
  dynamics_config.dt_internal =
      nh_.param<double>("dynamics/substep_size", 0.001);

  auto dynamics = std::make_shared<PathfinderDynamics>(dynamics_config);

  // -------------------------------
  // cost
  // -------------------------------
  auto cost = std::make_shared<PathfinderCost>();

  // -------------------------------
  // config
  // -------------------------------
  std::string config_file =
      ros::package::getPath("mppi_pathfinder") + "/config/params.yaml";
  if (!config_.init_from_file(config_file)) {
    ROS_ERROR_STREAM("Failed to init solver options from " << config_file);
    return false;
  }

  // -------------------------------
  // controller
  // -------------------------------
  controller = std::make_shared<mppi::PathIntegral>(dynamics, cost, config_);

  // -------------------------------
  // initialize reference
  // -------------------------------
  // cart at the goal position
  ref_.rr.resize(1,
                 mppi::observation_t::Zero(PathfinderDim::REFERENCE_DIMENSION));
  ref_.rr[0](0) = 2.0;
  ref_.tt.resize(1, 0.0);
  optimal_rollout_publisher_ =
      nh_.advertise<geometry_msgs::PoseArray>("/optimal_rollout", 1);
  return true;
}

bool PathfinderControllerInterface::update_reference() {
  if (!reference_set_) get_controller()->set_reference_trajectory(ref_);
  reference_set_ = true;
  return true;
}

void PathfinderControllerInterface::publish_ros() {}

void PathfinderControllerInterface::publish_optimal_rollout() {
  get_controller()->get_optimal_rollout(optimal_rollout_states_,
                                        optimal_rollout_inputs_);
  geometry_msgs::PoseArray optimal_rollout_array;
  geometry_msgs::Pose current_pose;

  optimal_rollout_array.header.frame_id = "odom";
  optimal_rollout_array.header.stamp = ros::Time::now();
  for (int i = 0; i < optimal_rollout_states_.size(); i++) {
    current_pose.position.x = optimal_rollout_states_[i](2);
    current_pose.position.y = 0;
    current_pose.position.z = 0;

    optimal_rollout_array.poses.push_back(current_pose);
  }
  optimal_rollout_publisher_.publish(optimal_rollout_array);
}
