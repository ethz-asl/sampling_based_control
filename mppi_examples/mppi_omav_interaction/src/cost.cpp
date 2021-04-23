/*!
 * @file     cost.cpp
 * @author   Matthias Studiger
 * @date     10.04.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_omav_interaction/cost.h"
#include "mppi_pinocchio/model.h"
#include <Eigen/Geometry>
#include <algorithm>

using namespace omav_interaction;

OMAVInteractionCost::OMAVInteractionCost(const std::string &robot_description,
                                         const OMAVInteractionCostParam &param)
    : param_(param), robot_description_(robot_description) {}

double
OMAVInteractionCost::distance_from_obstacle_cost(const mppi::observation_t &x) {
  distance = std::sqrt(std::pow(x(0) - param_.x_obstacle, 2) +
                       std::pow(x(1) - param_.y_obstacle, 2));
  distance_from_savezone = (distance - (2 + 1));
  obstacle_cost = -param_.Q_obstacle * std::min(0.0, distance_from_savezone);
  return obstacle_cost;
}

mppi::CostBase::cost_t
OMAVInteractionCost::compute_cost(const mppi::observation_t &x,
                                  const mppi::reference_t &ref,
                                  const double t) {
  double cost = 0.0;
  mode = ref(14);

  if (mode == 0) {
    ROS_INFO_STREAM_ONCE("Position Mode activated");
    // Leafing Field Cost
    if (abs(x(0)) > param_.x_limit || abs(x(1)) > param_.y_limit ||
        abs(x(2)) > param_.z_limit) {
      cost += param_.Q_leafing_field;
    }
    // Pose Cost
    mppi_pinocchio::Pose current_pose, reference_pose;
    current_pose.translation = x.head<3>();
    current_pose.rotation = {x(3), x(4), x(5), x(6)};
    reference_pose.translation = ref.head<3>();
    reference_pose.rotation = {ref(3), ref(4), ref(5), ref(6)};

    delta_pose = mppi_pinocchio::get_delta(current_pose, reference_pose);
    cost += delta_pose.transpose() * param_.Q_pose * delta_pose;

    // cost += OMAVInteractionCost::distance_from_obstacle_cost(x);
  }
  if (mode == 0) {
    ROS_INFO_STREAM_ONCE("Touch Mode activated");
    mppi_pinocchio::Pose object_pose, reference_pose_object;
    object_pose.translation = x.segment<3>(13);
    std::cout << "----------------------------" << std::endl;
    std::cout << object_pose.translation << std::endl;
    object_pose.rotation = {x(16), x(17), x(18), x(19)};
    // std::cout << x(16) << std::endl;
    reference_pose_object.translation = ref.segment<3>(7);
    reference_pose_object.rotation = {ref(10), ref(11), ref(13), ref(13)};
    delta_pose_object =
        mppi_pinocchio::get_delta(object_pose, reference_pose_object);
    std::cout << cost << std::endl;
    cost += 1000 * delta_pose_object.transpose() * param_.Q_pose *
            delta_pose_object;
    std::cout << cost << std::endl;
  }

  return cost;
}

bool OMAVInteractionCostParam::parse_from_ros(const ros::NodeHandle &nh) {
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

std::ostream &
operator<<(std::ostream &os,
           const omav_interaction::OMAVInteractionCostParam &param) {
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
    os << " orientation_cost: " << param.Q_orientation << std::endl;
    os << " obstacle_cost: " << param.Q_obstacle << std::endl;
    os << " obstacle_position_x: " << param.x_obstacle << std::endl;
    os << " obstacle_position_y: " << param.y_obstacle << std::endl;

    return os;
}
