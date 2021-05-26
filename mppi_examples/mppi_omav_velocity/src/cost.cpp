/*!
 * @file     cost.cpp
 * @author   Matthias Studiger
 * @date     10.04.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_omav_velocity/cost.h"
#include "mppi_pinocchio/model.h"
#include <Eigen/Geometry>
#include <algorithm>

using namespace omav_velocity;

OMAVVelocityCost::OMAVVelocityCost(const std::string &robot_description,
                                   const OMAVVelocityCostParam &param)
    : param_(param), robot_description_(robot_description) {}

double OMAVVelocityCost::distance_from_obstacle_cost(
    const mppi::observation_t &x, float x_obstacle, float y_obstacle) {
  distance_ = std::sqrt(std::pow(x(0) - x_obstacle, 2) +
                        std::pow(x(1) - y_obstacle, 2));
  distance_from_savezone_ = (distance_ - (2 + 1));
  obstacle_cost_ = -param_.Q_obstacle * std::min(0.0, distance_from_savezone_);
  return obstacle_cost_;
}

mppi::CostBase::cost_t
OMAVVelocityCost::compute_cost(const mppi::observation_t &x,
                               const mppi::reference_t &ref, const double t) {
  // Initialize Cost
  double cost = 0.0;
  // Get mode
  mode_ = ref(8);

  // Leafing Field Cost
  if (x(0) > param_.x_limit_max || x(1) > param_.y_limit_max ||
      x(2) > param_.z_limit_max || x(0) < param_.x_limit_min ||
      x(1) < param_.y_limit_min) {
    cost += param_.Q_leafing_field;
  }

  if ((x(2) - param_.floor_thresh) < 0 && ref(7)) {
    cost += -param_.Q_floor / param_.floor_thresh * x(2) + param_.Q_floor;
  }
  if (mode_ == 0) {
    // Pose Cost
    mppi_pinocchio::Pose current_pose, reference_pose;
    current_pose.translation = x.head<3>();
    current_pose.rotation = {x(3), x(4), x(5), x(6)};
    reference_pose.translation = ref.head<3>();
    reference_pose.rotation = {ref(3), ref(4), ref(5), ref(6)};

    delta_pose_ = mppi_pinocchio::get_delta(current_pose, reference_pose);
    cost += delta_pose_.transpose() * param_.Q_pose * delta_pose_;

    // cost += OMAVVelocityCost::distance_from_obstacle_cost(x, ref(9),
    // ref(10));
  }
  if (mode_ == 1) {
    cost += 10 * std::pow(ref(11) - x(13), 2);
  }

  return cost;
}

bool OMAVVelocityCostParam::parse_from_ros(const ros::NodeHandle &nh) {
  if (!nh.getParam("x_distance_weight", Q_distance_x) || Q_distance_x < 0) {
    ROS_ERROR("Failed to parse x_distance_cost or invalid!");
    return false;
  }
  if (!nh.getParam("y_distance_weight", Q_distance_y) || Q_distance_y < 0) {
    ROS_ERROR("Failed to parse y_distance_cost or invalid!");
    return false;
  }
  if (!nh.getParam("z_distance_weight", Q_distance_z) || Q_distance_z < 0) {
    ROS_ERROR("Failed to parse z_distance_cost or invalid!");
    return false;
  }

  if (!nh.getParam("leafing_field_cost", Q_leafing_field) ||
      Q_leafing_field < 0) {
    ROS_ERROR("Failed to parse leafing_field_cost or invalid!");
    return false;
  }
  if (!nh.getParam("field_limit_x_min", x_limit_min)) {
    ROS_ERROR("Failed to parse field_limit_x_min or invalid!");
    return false;
  }
  if (!nh.getParam("field_limit_x_max", x_limit_max)) {
    ROS_ERROR("Failed to parse field_limit_x_max or invalid!");
    return false;
  }
  if (!nh.getParam("field_limit_y_min", y_limit_min)) {
    ROS_ERROR("Failed to parse field_limit_y_min or invalid!");
    return false;
  }
  if (!nh.getParam("field_limit_y_max", y_limit_max)) {
    ROS_ERROR("Failed to parse field_limit_y_max or invalid!");
    return false;
  }
  if (!nh.getParam("field_limit_z_max", z_limit_max) || z_limit_max < 0) {
    ROS_ERROR("Failed to parse field_limit_z_max or invalid!");
    return false;
  }
  if (!nh.getParam("floor_threshold", floor_thresh) || floor_thresh < 0) {
    ROS_ERROR("Failed to parse floor_thresh or invalid!");
    return false;
  }
  if (!nh.getParam("floor_cost", Q_floor) || Q_floor < 0) {
    ROS_ERROR("Failed to parse field_limit_z or invalid!");
    return false;
  }
  if (!nh.getParam("orientation_weight", Q_orientation) || Q_orientation < 0) {
    ROS_ERROR("Failed to parse Orientation_cost or invalid!");
    return false;
  }
  if (!nh.getParam("obstacle_cost", Q_obstacle) || Q_obstacle < 0) {
    ROS_ERROR("Failed to parse obstacle_cost or invalid!");
    return false;
  }
  if (!nh.getParam("obstacle_position_x", x_obstacle)) {
    ROS_ERROR("Failed to parse obstacle_position_x or invalid!");
    return false;
  }
  if (!nh.getParam("obstacle_position_y", y_obstacle)) {
    ROS_ERROR("Failed to parse obstacle_position_y or invalid!");
    return false;
  }

  pose_costs << Q_distance_x, Q_distance_y, Q_distance_y, Q_orientation,
      Q_orientation, Q_orientation;
  Q_pose = pose_costs.asDiagonal();

  return true;
}
std::ostream &operator<<(std::ostream &os,
                         const omav_velocity::OMAVVelocityCostParam &param) {
  // clang-format off
    os << "========================================" << std::endl;
    os << "         OMAV Cost Parameters           " << std::endl;
    os << "========================================" << std::endl;
    os << " x_distance_weight: " << param.Q_distance_x << std::endl;
    os << " y_distance_weight: " << param.Q_distance_y << std::endl;
    os << " z_distance_weight: " << param.Q_distance_z << std::endl;
    os << " leafing_field_cost: " << param.Q_leafing_field << std::endl;
    os << " field_limit_x_min: " << param.x_limit_min << std::endl;
    os << " field_limit_x_max: " << param.x_limit_max << std::endl;
    os << " field_limit_y_min: " << param.y_limit_min << std::endl;
    os << " field_limit_y_max: " << param.y_limit_max << std::endl;
    os << " field_limit_z_max: " << param.z_limit_max << std::endl;
    os << " floor_threshold: " << param.floor_thresh << std::endl;
    os << " floor_cost: " << param.Q_floor << std::endl;
    os << " orientation_cost: " << param.Q_orientation << std::endl;
    os << " obstacle_cost: " << param.Q_obstacle << std::endl;
    os << " obstacle_position_x: " << param.x_obstacle << std::endl;
    os << " obstacle_position_y: " << param.y_obstacle << std::endl;

    return os;
}
