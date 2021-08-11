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

  if (!nh.param("safety_filter/first_derivative_limits",
                first_derivative_limits, false)) {
    ROS_WARN("Failed to parse safety_filter/first_derivative_limits");
    return false;
  }

  if (first_derivative_limits) {
    std::vector<double> ud_min_v, ud_max_v;
    if (!nh.param<std::vector<double>>("safety_filter/ud_min", ud_min_v, {})) {
      ROS_WARN("Failed to parse safety_filter/ud_min");
      return false;
    }

    if (!nh.param<std::vector<double>>("safety_filter/ud_max", ud_max_v, {})) {
      ROS_WARN("Failed to parse safety_filter/ud_max");
      return false;
    }

    int u_size = ud_min_v.size();
    if (u_size != ud_max_v.size()) {
      throw std::runtime_error("First derivative limits have different size!");
    }

    ud_min.resize(u_size);
    ud_max.resize(u_size);
    for (int i = 0; i < u_size; i++) {
      ud_min[i] = ud_min_v[i];
      ud_max[i] = ud_max_v[i];
    }
  }

  if (!nh.param("safety_filter/second_derivative_limits",
                second_derivative_limits, false)) {
    ROS_WARN("Failed to parse safety_filter/second_derivative_limits");
    return false;
  }

  if (second_derivative_limits) {
    std::vector<double> udd_min_v, udd_max_v;
    if (!nh.param<std::vector<double>>("safety_filter/udd_min", udd_min_v,
                                       {})) {
      ROS_WARN("Failed to parse safety_filter/udd_min");
      return false;
    }

    if (!nh.param<std::vector<double>>("safety_filter/ud_max", udd_max_v, {})) {
      ROS_WARN("Failed to parse safety_filter/udd_max");
      return false;
    }

    int u_size = udd_min_v.size();
    if (u_size != udd_max_v.size()) {
      throw std::runtime_error("Second derivative limits have different size!");
    }

    udd_min.resize(u_size);
    udd_max.resize(u_size);
    for (int i = 0; i < u_size; i++) {
      udd_min[i] = udd_min_v[i];
      udd_max[i] = udd_max_v[i];
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
    if (!nh.param("safety_filter/passivity_constraint/min_tank_energy",
                  min_tank_energy, 0.0)) {
      ROS_WARN(
          "Failed to parse safety_filter/passivity_constraint/min_tank_energy");
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

  os << "first_derivative_limits: " << settings.first_derivative_limits << std::endl;
  os << " >> ud_min=" << settings.ud_min.transpose() << std::endl;
  os << " >> ud_max=" << settings.ud_max.transpose() << std::endl;

  os << "second_derivative_limits: " << settings.second_derivative_limits << std::endl;
  os << " >> udd_min=" << settings.udd_min.transpose() << std::endl;
  os << " >> udd_max=" << settings.udd_max.transpose() << std::endl;

  os << "joint_limits: " << settings.joint_limits << std::endl;
  os << " >> q_min=" << settings.q_min.transpose() << std::endl;
  os << " >> q_max=" << settings.q_max.transpose() << std::endl;

  os << "cartesian_limits: " << settings.cartesian_limits << std::endl;
  os << " >> max_reach: " << settings.max_reach << std::endl;
  os << " >> min_distance: " << settings.min_dist << std::endl;

  os << "passivity_constraint: " << settings.passivity_constraint << std::endl;
  os << " >> min tank energy: " << settings.min_tank_energy << std::endl;
  os << "verbose: " << settings.verbose << std::endl;
  // clang-format on
  return os;
}
