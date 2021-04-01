/*!
 * @file     cost.cpp
 * @author   Matthias Studiger
 * @date     22.03.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_omav_raisim/cost.h"
#include "mppi_pinocchio/model.h"
#include <Eigen/Geometry>
#include <algorithm>

using namespace omav_raisim;

OMAVRaisimCost::OMAVRaisimCost(const std::string &robot_description,
                               const OMAVRaisimCostParam &param)
    : param_(param), robot_description_(robot_description) {}

mppi::CostBase::cost_t
OMAVRaisimCost::compute_cost(const mppi::observation_t &x,
                             const mppi::reference_t &ref, const double t) {
  double cost = 0.0;

  // Leafing Field Cost
  if (abs(x(16)) > param_.x_limit || abs(x(17)) > param_.y_limit ||
      abs(x(18)) > param_.z_limit) {
    cost += param_.Q_leafing_field;
  }
  // Pose Cost
  mppi_pinocchio::Pose current_pose, reference_pose;
  current_pose.translation = x.tail<3>();
  current_pose.rotation = {x(9), x(10), x(11), x(12)};
  reference_pose.translation = ref.head<3>();
  reference_pose.rotation = {ref(3), ref(4), ref(5), ref(6)};

  delta_pose = mppi_pinocchio::get_delta(current_pose, reference_pose);
  cost += delta_pose.transpose() * param_.Q_pose * delta_pose;

  // Maximum Velocity cost
  cost += std::max(0.0, std::sqrt(pow(x(6), 2) + pow(x(7), 2) + pow(x(8), 2)) *
                                param_.Q_velocity_max -
                            param_.max_velocity * param_.Q_velocity_max);
  // Maximum angular velocity cost
  cost +=
      param_.Q_omega * std::sqrt(pow(x(13), 2) + pow(x(14), 2) + pow(x(15), 2));
  // Maximum Force cost
  cost += std::max(0.0, param_.Q_thrust_max * x(0) -
                            param_.max_thrust * param_.Q_thrust_max);
  cost += std::max(0.0, param_.Q_thrust_max * x(1) -
                            param_.max_thrust * param_.Q_thrust_max);
  cost += std::max(0.0, param_.Q_thrust_max * x(2) -
                            param_.max_thrust * param_.Q_thrust_max);

  return cost;
}

bool OMAVRaisimCostParam::parse_from_ros(const ros::NodeHandle &nh) {
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
  if (!nh.getParam("field_limit_x", x_limit) || x_limit < 0) {
    ROS_ERROR("Failed to parse field_limit_x or invalid!");
    return false;
  }
  if (!nh.getParam("field_limit_y", y_limit) || y_limit < 0) {
    ROS_ERROR("Failed to parse field_limit_y or invalid!");
    return false;
  }
  if (!nh.getParam("field_limit_z", z_limit) || z_limit < 0) {
    ROS_ERROR("Failed to parse field_limit_z or invalid!");
    return false;
  }
  if (!nh.getParam("maximum_velocity", max_velocity) || max_velocity < 0) {
    ROS_ERROR("Failed to parse maximum_velocity or invalid!");
    return false;
  }
  if (!nh.getParam("velocity_cost", Q_velocity_max) || Q_velocity_max < 0) {
    ROS_ERROR("Failed to parse velocity_cost or invalid!");
    return false;
  }
  if (!nh.getParam("maximum_thrust", max_thrust) || max_thrust < 0) {
    ROS_ERROR("Failed to parse maximum_thrust or invalid!");
    return false;
  }
  if (!nh.getParam("thrust_cost", Q_thrust_max) || Q_thrust_max < 0) {
    ROS_ERROR("Failed to parse thrust_cost or invalid!");
    return false;
  }
  if (!nh.getParam("orientation_weight", Q_orientation) || Q_orientation < 0) {
    ROS_ERROR("Failed to parse Orientation_cost or invalid!");
    return false;
  }
  if (!nh.getParam("omega_weight", Q_omega) || Q_omega < 0) {
    ROS_ERROR("Failed to parse omega_cost or invalid!");
    return false;
  }
  pose_costs << Q_distance_x, Q_distance_y, Q_distance_y, Q_orientation,
      Q_orientation, Q_orientation;
  Q_pose = pose_costs.asDiagonal();

  return true;
}
std::ostream &operator<<(std::ostream &os,
                         const omav_raisim::OMAVRaisimCostParam &param) {
  // clang-format off
    os << "========================================" << std::endl;
    os << "         OMAV Cost Parameters           " << std::endl;
    os << "========================================" << std::endl;
    os << " x_distance_weight: " << param.Q_distance_x << std::endl;
    os << " y_distance_weight: " << param.Q_distance_y << std::endl;
    os << " z_distance_weight: " << param.Q_distance_z << std::endl;
    os << " leafing_field_cost: " << param.Q_leafing_field << std::endl;
    os << " field_limit_x: " << param.x_limit << std::endl;
    os << " field_limit_y: " << param.y_limit << std::endl;
    os << " field_limit_z: " << param.z_limit << std::endl;
    os << " velocity_cost: " << param.Q_velocity_max << std::endl;
    os << " maximum_velocity: " << param.max_velocity << std::endl;
    os << " maximum_thrust: " << param.max_thrust << std::endl;
    os << " thrust_cost: " << param.Q_thrust_max << std::endl;
    os << " orientation_cost: " << param.Q_orientation << std::endl;
    os << " omega_cost: " << param.Q_omega << std::endl;

    return os;
}
