//
// Created by giuseppe on 09.08.21.
//

#include "mppi_sliding/params/dynamics_params.h"
#include "mppi_sliding/dimensions.h"

using namespace manipulation;

bool DynamicsParams::init_from_ros(ros::NodeHandle& nh, bool is_sim) {
  std::string prefix = (is_sim) ? "sim_" : "";

  if (!nh.getParam(prefix + "dynamics/dt", dt) || dt <= 0) {
    ROS_ERROR("Failed to parse dynamics/dt or invalid!");
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

  std::string mug_description_name =
      (is_sim) ? "/mug_description_raisim"
               : "/mug_description_raisim";

  if (!nh.getParam(mug_description_name, mug_description) ||
      mug_description.empty()) {
    ROS_ERROR_STREAM("Dynamics params parsing: failed to parse " << mug_description_name
                                        << " or invalid!");
    return false;
  }

  //parse geometry params
  if (!nh.getParam("geometry/cylinder_height",cylinder_height))
  {
    ROS_ERROR_STREAM("geometry params parsing: failed to parse geometry/cylinder_height or invalid!");
    return false;
  }

  if (!nh.getParam("geometry/cylinder_radius",cylinder_radius))
  {
    ROS_ERROR_STREAM("geometry params parsing: failed to parse geometry/cylinder_radius or invalid!");
    return false;
  }

  if (!nh.getParam("geometry/cylinder_z",cylinder_z))
  {
    ROS_ERROR_STREAM("geometry params parsing: failed to parse geometry/cylinder_z or invalid!");
    return false;
  }

  if (!nh.getParam("geometry/cylinder_mass",cylinder_mass))
  {
    ROS_ERROR_STREAM("geometry params parsing: failed to parse geometry/cylinder_mass or invalid!");
    return false;
  }

  if (!nh.getParam("geometry/table_position",table_position))
  {
    ROS_ERROR_STREAM("geometry params parsing: failed to parse geometry/table_position or invalid!");
    return false;
  }

  if (!nh.getParam("geometry/target_pos",target_pos))
  {
    ROS_ERROR_STREAM("geometry params parsing: failed to parse geometry/target_pos or invalid!");
    return false;
  }

  if (!nh.getParam("geometry/friction",friction))
  {
    ROS_ERROR_STREAM("geometry params parsing: failed to parse geometry/friction or invalid!");
    return false;
  }

  //parse control config params
  if (!nh.getParam("setup_config/fixed_base",fixed_base))
  {
    ROS_ERROR_STREAM("config params parsing: failed to parse setup_config/fixed_base or invalid!");
    return false;
  }

  if (!nh.getParam("setup_config/update_geometry",update_geometry))
  {
    ROS_ERROR_STREAM("config params parsing: failed to parse setup_config/update_geometry or invalid!");
    return false;
  }


  // parse gains
  if (!gains.init_from_ros(nh, prefix + "dynamics/")) {
    ROS_ERROR("Failed to parse simulation gains.");
    return false;
  }

  return true;
}

std::ostream& operator<<(std::ostream& os,
                         const manipulation::DynamicsParams& params) {
  // clang-format off
  os << "Dynamics params" << std::endl;
  os << "dt=" << params.dt << std::endl;
  os << "dynamics gains: " << std::endl;
  os << params.gains;
  return os;
  // clang-format on
}
