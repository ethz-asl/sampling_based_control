//
// Created by giuseppe on 09.08.21.
//

#include "mppi_manipulation/dimensions.h"
#include "mppi_manipulation/params/dynamics_params.h"

using namespace manipulation;

bool DynamicsParams::init_from_ros(ros::NodeHandle& nh, bool is_sim) {
  std::string prefix = (is_sim) ? "sim_" : "";

  if (!nh.getParam(prefix + "dynamics/dt", dt) || dt <= 0) {
    ROS_ERROR("Failed to parse dynamics/dt or invalid!");
    return false;
  }

  if (!nh.getParam(prefix + "dynamics/has_filter", has_filter)) {
    ROS_ERROR("Filed to parse dynamics/has_filter or invalid!");
    return false;
  }

  if (!nh.getParam(prefix + "dynamics/apply_filter", apply_filter)) {
    ROS_ERROR("Failed to parse dynamics/apply_filter or invalid!");
    return false;
  }

  std::vector<double> x0;
  if (!nh.param<std::vector<double>>(prefix + "dynamics/initial_state", x0,
                                     {}) ||
      x0.empty()) {
    ROS_ERROR("Failed to parse dynamics/initial_state or invalid");
    return false;
  }
  if (x0.size() != STATE_DIMENSION){
    ROS_ERROR_STREAM("Initial state with the wrong dimension: " << initial_state.size() << "!=" << (int)STATE_DIMENSION);
    return false;
  }

  initial_state.setZero(x0.size());
  for (int i = 0; i < x0.size(); i++) initial_state(i) = x0[i];

  if (!nh.getParam("/robot_description_raisim", robot_description) ||
      robot_description.empty()) {
    ROS_ERROR("Failed to parse /robot_description_raisim or invalid!");
    return false;
  }

  if (!nh.getParam("/object_description_raisim", object_description) ||
      object_description.empty()) {
    ROS_ERROR("Failed to parse /object_description_raisim or invalid!");
    return false;
  }

  if (!gains.init_from_ros(nh, prefix)) {
    ROS_ERROR("Failed to parse simulation gains.");
    return false;
  }

  if (!filter_params.init_from_ros(nh) && has_filter) {
    ROS_ERROR("Failed to parse filter parameters.");
    return false;
  }
  return true;
}

std::ostream& operator<<(std::ostream& os,
                         const manipulation::DynamicsParams& params) {
  // clang-format off
  os << "Dynamics params" << std::endl;
  os << "dt=" << params.dt << std::endl;
  os << "has filter = " << params.has_filter << std::endl;
  os << "apply filter = " << params.apply_filter << std::endl;
  os << params.gains << std::endl;
  return os;
  // clang-format on
}
