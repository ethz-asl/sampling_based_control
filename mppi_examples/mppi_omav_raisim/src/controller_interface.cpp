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
  double x_goal_position, y_goal_position, z_goal_position;
  nh_.param<double>("goal_position_x", x_goal_position, 0.0);
  nh_.param<double>("goal_position_y", y_goal_position, 0.0);
  nh_.param<double>("goal_position_z", z_goal_position, 0.0);
  ref_.rr.resize(1, mppi::observation_t::Zero(3));
  ref_.rr[0](0) = x_goal_position;
  ref_.rr[0](1) = y_goal_position;
  ref_.rr[0](2) = z_goal_position;
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
