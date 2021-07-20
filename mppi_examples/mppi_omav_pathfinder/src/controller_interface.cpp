/*!
 * @file     controller_interface.cpp
 * @author   Matthias Studiger
 * @date     15.03.2021
 * @version  1.0
 * @brief    description
 */

#include "mppi_omav_pathfinder/controller_interface.h"
#include <ros/package.h>

using namespace omav_pathfinder;

bool OMAV_PathfinderControllerInterface::set_controller(
    std::shared_ptr<mppi::PathIntegral> &controller) {
  // -------------------------------
  // dynamics
  // -------------------------------
  auto dynamics_config = OMAV_PathfinderDynamicsConfig();
  dynamics_config.mass = nh_.param<double>("dynamics/mass", 4.04);
  dynamics_config.gravity = nh_.param<double>("dynamics/g", 9.8086);
  dynamics_config.Ix = nh_.param<double>("dynamics/Ix", 0.078359);
  dynamics_config.Iy = nh_.param<double>("dynamics/Iy", 0.081797);
  dynamics_config.Iz = nh_.param<double>("dynamics/Iz", 0.153355);
  dynamics_config.dt_internal =
      nh_.param<double>("dynamics/substep_size", 0.001);

  auto dynamics = std::make_shared<OMAV_PathfinderDynamics>(dynamics_config);

  // -------------------------------
  // cost
  // -------------------------------
  auto cost = std::make_shared<OMAV_PathfinderCost>();

  // -------------------------------
  // config
  // -------------------------------
  std::string config_file =
      ros::package::getPath("mppi_OMAV_Pathfinder") + "/config/params.yaml";
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
                 mppi::observation_t::Zero(OMAV_PathfinderDim::REFERENCE_DIMENSION));
  ref_.rr[0](0) = 20.0;
  ref_.rr[0](1) = 3.0;
  ref_.rr[0](2) = 10.0;
  ref_.tt.resize(1, 0.0);
  return true;
}

bool OMAV_PathfinderControllerInterface::update_reference() {
  if (!reference_set_) get_controller()->set_reference_trajectory(ref_);
  reference_set_ = true;
  return true;
}

void OMAV_PathfinderControllerInterface::publish_ros() {}