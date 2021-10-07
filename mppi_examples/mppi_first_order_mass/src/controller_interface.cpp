/*!
 * @file     controller_ros.cpp
 * @author   Giuseppe Rizzi
 * @date     03.09.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_first_order_mass/controller_interface.h"
#include <geometry_msgs/Point.h>
#include <mppi/policies/gaussian_policy.h>
#include <mppi/policies/spline_policy.h>
#include <ros/package.h>

using namespace fom;

FOMControllerInterface::FOMControllerInterface(ros::NodeHandle& nh)
    : ControllerRos(nh) {
  reference_subscriber_ =
      nh.subscribe("/goal", 1, &FOMControllerInterface::goal_cb, this);
};

bool FOMControllerInterface::set_controller(mppi::solver_ptr& controller) {
  // -------------------------------
  // config
  // -------------------------------
  std::string config_file =
      ros::package::getPath("mppi_first_order_mass") + "/config/params.yaml";
  if (!config_.init_from_file(config_file)) {
    ROS_ERROR_STREAM("Failed to init solver options from " << config_file);
    return false;
  }

  // -------------------------------
  // dynamics
  // -------------------------------
  auto dynamics = std::make_shared<FOMDynamics>(config_.step_size);

  // -------------------------------
  // cost
  // -------------------------------
  auto cost = std::make_shared<FOMCost>();

  // -------------------------------
  // policy
  // -------------------------------
  bool gaussian_policy;
  nh_.param<bool>("gaussian_policy", gaussian_policy, true);
  std::shared_ptr<mppi::Policy> policy;
  if (gaussian_policy) {
    policy = std::make_shared<mppi::GaussianPolicy>(
        int(FOMDim::INPUT_DIMENSION), config_);
  } else {
    policy = std::make_shared<mppi::SplinePolicy>(int(FOMDim::INPUT_DIMENSION),
                                                  config_);
  }

  // -------------------------------
  // controller
  // -------------------------------
  controller = std::make_shared<mppi::Solver>(dynamics, cost, policy, config_);

  // -------------------------------
  // initialize reference
  // -------------------------------
  // cart at the origin, pole up
  ref_.rr.resize(1, mppi::observation_t::Zero(FOMDim::REFERENCE_DIMENSION));
  ref_.tt.resize(1, 0.0);
  reference_set_ = false;
  return true;
}

void FOMControllerInterface::goal_cb(const geometry_msgs::PointConstPtr& msg) {
  ref_.rr[0](0) = msg->x;
  ref_.rr[0](1) = msg->y;
  reference_set_ = true;
}

bool FOMControllerInterface::update_reference() {
  if (reference_set_) {
    get_controller()->set_reference_trajectory(ref_);
    reference_set_ = false;
  }
  return true;
}

void FOMControllerInterface::publish_ros() {}
