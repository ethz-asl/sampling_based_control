//
// Created by giuseppe on 09.08.21.
//

#include "mppi_manipulation/params/dynamics_params.h"
#include "mppi_manipulation/dimensions.h"

using namespace manipulation;
bool DynamicsParams::init_from_config(const manipulation::Config& config) {
  dt = config.dt;
  robot_description = config.robot_description_raisim;
  object_description = config.object_description_raisim;
  gains = config.gains;
  initial_state = config.initial_state;
  raisim_res_path = config.raisim_res_path;
  ignore_object_self_collision = config.ignore_object_self_collision;
  debug_prints = config.debug_prints;

  articulation_joint = config.articulation_joint;
  object_handle_link = config.object_handle_link;
  object_handle_joint = config.object_handle_joint;

  return true;
}


bool DynamicsParams::init_from_ros(ros::NodeHandle& nh, bool is_sim) {
  std::string prefix = (is_sim) ? "sim_" : "";

  if (!nh.getParam(prefix + "dynamics/dt", dt) || dt <= 0) {
    ROS_ERROR("Failed to parse dynamics/dt or invalid!");
    return false;
  }

  if (!nh.getParam(prefix + "dynamics/object_handle_link",
                   object_handle_link)) {
    ROS_ERROR("Failed to parse dynamics/object_handle_link or invalid!");
    return false;
  }

  if (!nh.getParam(prefix + "dynamics/object_handle_joint",
                   object_handle_joint)) {
    ROS_ERROR("Failed to parse dynamics/object_handle_joint");
  }

  if (!nh.getParam(prefix + "dynamics/articulation_joint",
                   articulation_joint)) {
    ROS_ERROR("Failed to parse dynamics/articulation_joint or invalid!");
    return false;
  }

  std::vector<double> x0;
  if (!nh.param<std::vector<double>>(prefix + "dynamics/initial_state", x0,
                                     {}) ||
      x0.empty()) {
    ROS_ERROR("Failed to parse dynamics/initial_state or invalid");
    return false;
  }
  if (x0.size() != STATE_DIMENSION) {
    ROS_ERROR_STREAM("Initial state with the wrong dimension: "
                     << initial_state.size() << "!=" << (int)STATE_DIMENSION);
    return false;
  }

  initial_state.setZero(x0.size());
  for (int i = 0; i < x0.size(); i++) initial_state(i) = x0[i];

  if (!nh.getParam("/robot_description_raisim", robot_description) ||
      robot_description.empty()) {
    ROS_ERROR("Failed to parse /robot_description_raisim or invalid!");
    return false;
  }

  std::string object_description_name =
      (is_sim) ? "/object_description_raisim_simulation"
               : "/object_description_raisim";

  if (!nh.getParam(object_description_name, object_description) ||
      object_description.empty()) {
    ROS_ERROR_STREAM("Dynamics params parsing: failed to parse " << object_description_name
                                        << " or invalid!");
    return false;
  }

  if (!gains.init_from_ros(nh, prefix + "dynamics/")) {
    ROS_ERROR("Failed to parse simulation gains.");
    return false;
  }

  return true;
}

std::ostream& operator<<(std::ostream& os,
                         const manipulation::DynamicsParams& params) {
  if (params.debug_prints) {
    // clang-format off
    os << "Dynamics params" << std::endl;
    os << "dt=" << params.dt << std::endl;
    os << "dynamics gains: " << std::endl;
    os << params.gains;
    os << "object handle link = " << params.object_handle_link << std::endl;
    os << "object handle joint = " << params.object_handle_joint << std::endl;
    os << "articulation joint = " << params.articulation_joint << std::endl;
  }else{
    os << "";
  }
  return os;
  // clang-format on
}
