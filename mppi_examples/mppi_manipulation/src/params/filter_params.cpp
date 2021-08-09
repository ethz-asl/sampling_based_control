//
// Created by giuseppe on 09.08.21.
//

#include "mppi_manipulation/params/filter_params.h"

using namespace manipulation;

bool FilterParams::init_from_ros(ros::NodeHandle& nh) {
  if (!nh.param<std::string>("/robot_description_safety_filter", urdf, "") ||
      urdf.empty()) {
    ROS_ERROR("Failed to parse /robot_description_safety_filter or invalid.");
    return false;
  }

  if (!nh.param("safety_filter/verbose", verbose, false)) {
    ROS_WARN("Failed to parse safety_filter/verbose");
    return false;
  }

  if (!nh.param("safety_filter/joint_limits", joint_limits, false)) {
    ROS_WARN("Failed to parse safety_filter/joint_limits");
    return false;
  }

  if (joint_limits) {
    std::vector<double> q_min_v, q_max_v;
    if (!nh.param<std::vector<double>>("safety_filter/q_min", q_min_v, {})) {
      ROS_WARN("Failed to parse safety_filter/q_min");
      return false;
    }

    if (!nh.param<std::vector<double>>("safety_filter/q_max", q_max_v, {})) {
      ROS_WARN("Failed to parse safety_filter/q_max");
      return false;
    }

    int q_size = q_min_v.size();
    if (q_size != q_max_v.size()) {
      throw std::runtime_error("Joint limits have different size!");
    }

    q_min.resize(q_size);
    q_max.resize(q_size);
    for (int i = 0; i < q_size; i++) {
      q_min[i] = q_min_v[i];
      q_max[i] = q_max_v[i];
    }
  }

  if (!nh.param("safety_filter/input_limits", input_limits, false)) {
    ROS_WARN("Failed to parse safety_filter/input_limits");
    return false;
  }

  if (input_limits) {
    std::vector<double> u_min_v, u_max_v;
    if (!nh.param<std::vector<double>>("safety_filter/u_min", u_min_v, {})) {
      ROS_WARN("Failed to parse safety_filter/u_min");
      return false;
    }

    if (!nh.param<std::vector<double>>("safety_filter/u_max", u_max_v, {})) {
      ROS_WARN("Failed to parse safety_filter/u_max");
      return false;
    }

    int u_size = u_min_v.size();
    if (u_size != u_max_v.size()) {
      throw std::runtime_error("Joint limits have different size!");
    }

    u_min.resize(u_size);
    u_max.resize(u_size);
    for (int i = 0; i < u_size; i++) {
      u_min[i] = u_min_v[i];
      u_max[i] = u_max_v[i];
    }
  }

  if (!nh.param("safety_filter/cartesian_limits", cartesian_limits, false)) {
    ROS_WARN("Failed to parse safety_filter/cartesian_limits");
    return false;
  }

  if (cartesian_limits) {
    if (!nh.param("safety_filter/max_reach", max_reach, 0.80)) {
      ROS_WARN("Failed to parse safety_filter/max_reach");
      return false;
    }

    if (!nh.param("safety_filter/min_distance", min_dist, 0.15)) {
      ROS_WARN("Failed to parse safety_filter/min_distance");
      return false;
    }
  }

  if (!nh.param("safety_filter/passivity_constraint/active",
                passivity_constraint, false)) {
    ROS_WARN("Failed to parse safety_filter/passivity_constraint/active");
    return false;
  }

  if (passivity_constraint) {
    if (!nh.param("safety_filter/passivity_constraint/initial_energy",
                  tank_initial_energy, 1.0)) {
      ROS_WARN(
          "Failed to parse safety_filter/passivity_constraint/initial_energy");
      return false;
    }

    if (!nh.param("safety_filter/passivity_constraint/lower_bound",
                  tank_lower_energy_bound, 1e-3)) {
      ROS_WARN(
          "Failed to parse safety_filter/passivity_constraint/lower_bound");
      return false;
    }

    if (!nh.param("safety_filter/passivity_constraint/dt", tank_integration_dt,
                  0.01)) {
      ROS_WARN("Failed to parse safety_filter/passivity_constraint/dt");
      return false;
    }
  }

  return true;
}

std::ostream& operator<<(std::ostream& os, const FilterParams& settings) {
  // clang-format off
  os << "PandaMobileSafetyFilterSettings: " << std::endl;

  os << "input_limits: " << settings.input_limits << std::endl;
  os << " >> u_min=" << settings.u_min.transpose() << std::endl;
  os << " >> u_max=" << settings.u_max.transpose() << std::endl;

  os << "joint_limits: " << settings.joint_limits << std::endl;
  os << " >> q_min=" << settings.q_min.transpose() << std::endl;
  os << " >> q_max=" << settings.q_max.transpose() << std::endl;

  os << "cartesian_limits: " << settings.cartesian_limits << std::endl;
  os << " >> max_reach: " << settings.max_reach << std::endl;
  os << " >> min_distance: " << settings.min_dist << std::endl;

  os << "passivity_constraint: " << settings.passivity_constraint << std::endl;
  os << " >> tank_initial_energy: " << settings.tank_initial_energy << std::endl;
  os << " >> tank_lower_energy_bound: " << settings.tank_lower_energy_bound << std::endl;
  os << " >> tank_integration_dt: " << settings.tank_integration_dt << std::endl;

  os << "verbose: " << settings.verbose << std::endl;
  // clang-format on
  return os;
}
