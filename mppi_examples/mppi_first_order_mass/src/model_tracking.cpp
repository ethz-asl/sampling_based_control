/*!
 * @file     controller_ros.cpp
 * @author   Giuseppe Rizzi
 * @date     03.09.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_first_order_mass/model_tracking.h"
#include <geometry_msgs/Point.h>
#include <mppi/policies/gaussian_policy.h>
#include <mppi/policies/spline_policy.h>
#include <ros/package.h>

using namespace fom;

FOMModelTracking::FOMModelTracking(ros::NodeHandle& nh) : nh_(nh) {
  reference_subscriber_ =
      nh.subscribe("/goal", 1, &FOMModelTracking::goal_cb, this);
  state_publisher_ = nh.advertise<geometry_msgs::Point>("/state", 1);
  setup();
}

bool FOMModelTracking::setup() {
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
  // -------------------------------
  // initialize state
  // -------------------------------
  mppi::observation_t x0 = mppi::observation_t::Zero(2);

  // -------------------------------
  // controller
  // -------------------------------
  init(dynamics, cost, policy, x0, 0.0, config_);

  // -------------------------------
  // initialize reference
  // -------------------------------
  // cart at the origin, pole up
  ref_.rr.resize(1, mppi::observation_t::Zero(FOMDim::REFERENCE_DIMENSION));
  ref_.tt.resize(1, 0.0);
  return true;
}

void FOMModelTracking::goal_cb(const geometry_msgs::PointConstPtr& msg) {
  ref_.rr[0](0) = msg->x;
  ref_.rr[0](1) = msg->y;
  set_reference(ref_);
}

void FOMModelTracking::publish_ros() {
  state_.x = x_[0];
  state_.y = x_[1];
  state_publisher_.publish(state_);
}
