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
  // cart at the origin, pole up
  ref_.rr.resize(1,
                 mppi::observation_t::Zero(PathfinderDim::REFERENCE_DIMENSION));
  ref_.rr[0](0) = 5.0;
  ref_.rr[0](1) = 3.0;
  ref_.rr[0](2) = 0.0;
  ref_.tt.resize(1, 0.0);
  return true;
}

bool PathfinderControllerInterface::update_reference() {
  if (!reference_set_) get_controller()->set_reference_trajectory(ref_);
  reference_set_ = true;
  return true;
}

void PathfinderControllerInterface::publish_ros() {}