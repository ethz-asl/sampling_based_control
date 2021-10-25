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

  /////////////////////
  // Joint limits
  /////////////////////
  if (!nh.param("safety_filter/joint_limits/active", joint_limits, false)) {
    ROS_WARN("Failed to parse safety_filter/joint_limits");
    return false;
  }

  if (joint_limits) {
    if (!nh.param<bool>("safety_filter/joint_limits/soft", joint_limits_soft,
                        false)) {
      ROS_WARN("Failed to parse safety_filter/joint_limits/soft");
      return false;
    }

    if (!nh.param<double>("safety_filter/joint_limits/slack_multiplier",
                          joint_limits_slack_multiplier, 0.0)) {
      ROS_WARN("Failed to parse safety_filter/joint_limits/slack_multiplier");
      return false;
    }

    std::vector<double> q_min_v, q_max_v;
    if (!nh.param<std::vector<double>>("safety_filter/joint_limits/q_min",
                                       q_min_v, {})) {
      ROS_WARN("Failed to parse safety_filter/joint_limits/q_min");
      return false;
    }

    if (!nh.param<std::vector<double>>("safety_filter/joint_limits/q_max",
                                       q_max_v, {})) {
      ROS_WARN("Failed to parse safety_filter/joint_limits/q_max");
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

  /////////////////////
  // Input limits
  /////////////////////
  if (!nh.param("safety_filter/input_limits/active", input_limits, false)) {
    ROS_WARN("Failed to parse safety_filter/input_limits/active");
    return false;
  }

  if (input_limits) {
    std::vector<double> u_min_v, u_max_v;
    if (!nh.param<std::vector<double>>("safety_filter/input_limits/u_min",
                                       u_min_v, {})) {
      ROS_WARN("Failed to parse safety_filter/input_limits/u_min");
      return false;
    }

    if (!nh.param<std::vector<double>>("safety_filter/input_limits/u_max",
                                       u_max_v, {})) {
      ROS_WARN("Failed to parse safety_filter/input_limits/u_max");
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

  /////////////////////
  // First derivative limits
  /////////////////////
  if (!nh.param("safety_filter/first_derivative_limits/active",
                first_derivative_limits, false)) {
    ROS_WARN("Failed to parse safety_filter/first_derivative_limits/active");
    return false;
  }

  if (first_derivative_limits) {
    std::vector<double> ud_min_v, ud_max_v;
    if (!nh.param<std::vector<double>>(
            "safety_filter/first_derivative_limits/ud_min", ud_min_v, {})) {
      ROS_WARN("Failed to parse safety_filter/first_derivative_limits/ud_min");
      return false;
    }

    if (!nh.param<std::vector<double>>(
            "safety_filter/first_derivative_limits/ud_max", ud_max_v, {})) {
      ROS_WARN("Failed to parse safety_filter/first_derivative_limits/ud_max");
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

  /////////////////////
  // Object avoidance
  /////////////////////
  if (!nh.param("safety_filter/object_avoidance/active", object_avoidance,
                false)) {
    ROS_WARN("Failed to parse safety_filter/object_avoidance/active");
    return false;
  }

  if (object_avoidance) {
    if (!nh.param<bool>("safety_filter/object_avoidance/soft",
                        object_avoidance_soft, false)) {
      ROS_WARN("Failed to parse safety_filter/object_avoidance/soft");
      return false;
    }

    if (!nh.param<double>("safety_filter/object_avoidance/slack_multiplier",
                          object_avoidance_slack_multiplier, 0.0)) {
      ROS_WARN(
          "Failed to parse safety_filter/object_avoidance/slack_multiplier");
      return false;
    }

    if (!nh.param<std::string>("/robot_description_safety_filter", urdf, {})) {
      ROS_WARN("Failed to parse /robot_description_safety_filter");
      return false;
    }

    if (!nh.param<double>("safety_filter/object_avoidance/min_distance",
                          min_obstacle_distance, 0.0)) {
      ROS_WARN("safety_filter/object_avoidance/min_distance");
      return false;
    }

    if (!nh.param<std::string>("safety_filter/object_avoidance/object_frame_id",
                               object_frame_id, {})) {
      ROS_WARN("safety_filter/object_avoidance/object_frame_id");
      return false;
    }
  }

  /////////////////////
  // Obstacle avoidance
  /////////////////////
  if (!nh.param("safety_filter/obstacle_avoidance/active", obstacle_avoidance,
                false)) {
    ROS_WARN("Failed to parse safety_filter/obstacle_avoidance/active");
    return false;
  }

  if (obstacle_avoidance) {
    if (!nh.param<bool>("safety_filter/cartesian_limits/soft",
                        cartesian_limits_soft, false)) {
      ROS_WARN("Failed to parse safety_filter/cartesian_limits/soft");
      return false;
    }

    if (!nh.param<double>("safety_filter/cartesian_limits/slack_multiplier",
                          cartesian_limits_slack_multiplier, 0.0)) {
      ROS_WARN(
          "Failed to parse safety_filter/cartesian_limits/slack_multiplier");
      return false;
    }

    if (!nh.param<double>("safety_filter/obstacle_avoidance/min_distance",
                          min_obstacle_distance, 0.0)) {
      ROS_WARN("safety_filter/object_avoidance/min_distance");
      return false;
    }

    if (!nh.param<std::string>(
            "safety_filter/obstacle_avoidance/obstacle_frame_id",
            obstacle_frame_id, {})) {
      ROS_WARN("safety_filter/object_avoidance/obstacle_frame_id");
      return false;
    }
  }

  /////////////////////
  // Second derivative limits
  /////////////////////
  if (!nh.param("safety_filter/second_derivative_limits/active",
                second_derivative_limits, false)) {
    ROS_WARN("Failed to parse safety_filter/second_derivative_limits/active");
    return false;
  }

  if (second_derivative_limits) {
    std::vector<double> udd_min_v, udd_max_v;
    if (!nh.param<std::vector<double>>(
            "safety_filter/second_derivative_limits/udd_min", udd_min_v, {})) {
      ROS_WARN(
          "Failed to parse safety_filter/second_derivative_limits/udd_min");
      return false;
    }

    if (!nh.param<std::vector<double>>(
            "safety_filter/second_derivative_limits/udd_max", udd_max_v, {})) {
      ROS_WARN(
          "Failed to parse safety_filter/second_derivative_limits/udd_max");
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

  /////////////////////
  // Cartesian limits
  /////////////////////
  if (!nh.param("safety_filter/cartesian_limits/active", cartesian_limits,
                false)) {
    ROS_WARN("Failed to parse safety_filter/cartesian_limits/active");
    return false;
  }

  if (cartesian_limits) {
    if (!nh.param<bool>("safety_filter/cartesian_limits/soft",
                        cartesian_limits_soft, false)) {
      ROS_WARN("Failed to parse safety_filter/cartesian_limits/soft");
      return false;
    }

    if (!nh.param<double>("safety_filter/cartesian_limits/slack_multiplier",
                          cartesian_limits_slack_multiplier, 0.0)) {
      ROS_WARN(
          "Failed to parse safety_filter/cartesian_limits/slack_multiplier");
      return false;
    }

    if (!nh.param("safety_filter/cartesian_limits/max_reach", max_reach,
                  0.80)) {
      ROS_WARN("Failed to parse safety_filter/cartesian_limits/max_reach");
      return false;
    }

    if (!nh.param("safety_filter/cartesian_limits/min_distance", min_dist,
                  0.15)) {
      ROS_WARN("Failed to parse safety_filter/cartesian_limits/min_distance");
      return false;
    }
  }

  if (!nh.param("safety_filter/passivity_constraint/active",
                passivity_constraint, false)) {
    ROS_WARN("Failed to parse safety_filter/passivity_constraint/active");
    return false;
  }

  /////////////////////
  // Passivity constraints
  /////////////////////
  if (passivity_constraint) {
    if (!nh.param<bool>("safety_filter/passivity_constraint/soft",
                        passivity_constraint_soft, false)) {
      ROS_WARN("Failed to parse safety_filter/passivity_constraint/soft");
      return false;
    }

    if (!nh.param<double>("safety_filter/passivity_constraint/slack_multiplier",
                          passivity_constraint_slack_multiplier, 0.0)) {
      ROS_WARN(
          "Failed to parse "
          "safety_filter/passivity_constraint/slack_multiplier");
      return false;
    }

    if (!nh.param("safety_filter/passivity_constraint/min_tank_energy",
                  min_tank_energy, 0.0)) {
      ROS_WARN(
          "Failed to parse safety_filter/passivity_constraint/min_tank_energy");
      return false;
    }

    if (!nh.param("safety_filter/passivity_constraint/initial_tank_energy",
                  initial_tank_energy, 10.0)) {
      ROS_WARN(
          "Failed to parse "
          "safety_filter/passivity_constraint/initial_tank_energy");
      return false;
    }
  }

  return true;
}

std::ostream& operator<<(std::ostream& os, const FilterParams& settings) {
  // clang-format off
  os << "-----------------------------------------------" << std::endl;
  os << "PandaMobileSafetyFilterSettings: " << std::endl;
  os << "-----------------------------------------------" << std::endl;
  os << "Input limits" << std::endl;
  os << "active: "        << settings.input_limits << std::endl;
  os << "u_min: "         << settings.u_min.transpose() << std::endl;
  os << "u_max: "         << settings.u_max.transpose() << std::endl;
  os << "-----------------------------------------------" << std::endl;
  os << "First derivative limits" << std::endl;
  os << "active: "        << settings.first_derivative_limits << std::endl;
  os << "ud_min: "        << settings.ud_min.transpose() << std::endl;
  os << "ud_max: "        << settings.ud_max.transpose() << std::endl;
  os << "-----------------------------------------------" << std::endl;
  os << "Second derivative limits: " << std::endl;
  os << "active: "        << settings.second_derivative_limits << std::endl;
  os << "udd_min: "       << settings.udd_min.transpose() << std::endl;
  os << "udd_max: "       << settings.udd_max.transpose() << std::endl;
  os << "-----------------------------------------------" << std::endl;
  os << "Joint limits: " << std::endl;
  os << "active: "        << settings.joint_limits << std::endl;
  os << "soft: "          << settings.joint_limits_soft << std::endl;
  os << "slack_multiplier: " << settings.joint_limits_slack_multiplier << std::endl;
  os << "q_min: "         << settings.q_min.transpose() << std::endl;
  os << "q_max: "         << settings.q_max.transpose() << std::endl;
  os << "-----------------------------------------------" << std::endl;
  os << "Cartesian limits: " << std::endl;
  os << "active: "        << settings.cartesian_limits << std::endl;
  os << "soft: "          << settings.cartesian_limits_soft << std::endl;
  os << "slack_multiplier: " << settings.cartesian_limits_slack_multiplier << std::endl;
  os << "max_reach: "     << settings.max_reach << std::endl;
  os << "min_distance: "  << settings.min_dist << std::endl;
  os << "-----------------------------------------------" << std::endl;
  os << "Object avoidance: " << std::endl;
  os << "active: "        << settings.object_avoidance << std::endl;
  os << "soft: "          << settings.object_avoidance_soft << std::endl;
  os << "slack_multiplier: " << settings.object_avoidance_slack_multiplier << std::endl;
  os << "max_distance: "  << settings.min_object_distance << std::endl;
  os << "object_frame_id: " << settings.object_frame_id << std::endl; 
  os << "-----------------------------------------------" << std::endl;
  os << "Obstacle avoidance: " << std::endl;
  os << "active: "        << settings.obstacle_avoidance << std::endl;
  os << "soft: "          << settings.obstacle_avoidance_soft << std::endl;
  os << "slack_multiplier: " << settings.obstacle_avoidance_slack_multiplier << std::endl;
  os << "obstacle radius: " << settings.min_obstacle_distance << std::endl;
  os << "obstacle frame: " << settings.obstacle_frame_id << std::endl;
  os << "-----------------------------------------------" << std::endl;
  os << "Passivity constraint: " << std::endl;
  os << "active: "        << settings.passivity_constraint << std::endl;
  os << "soft: "          << settings.passivity_constraint_soft << std::endl;
  os << "slack_multiplier: " << settings.passivity_constraint_slack_multiplier << std::endl;
  os << "min_tank_energy: " << settings.min_tank_energy << std::endl;
  os << "initial_tank_energy: " << settings.initial_tank_energy << std::endl;
  os << "-----------------------------------------------" << std::endl;
  os << "Other" << std::endl;
  os << "verbose: "       << settings.verbose << std::endl;
  os << "-----------------------------------------------" << std::endl;

  // clang-format on
  return os;
}
