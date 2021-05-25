/*!
 * @file     cost.cpp
 * @author   Matthias Studiger
 * @date     10.04.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_omav_interaction/cost.h"
#include <Eigen/Geometry>
#include <algorithm>

using namespace omav_interaction;

OMAVInteractionCost::OMAVInteractionCost(
    const std::string &robot_description,
    const std::string &robot_description_pinocchio,
    const std::string &object_description, OMAVInteractionCostParam *param)
    : robot_description_(robot_description),
      robot_description_pinocchio_(robot_description_pinocchio),
      object_description_(object_description) {

  param_ptr_ = param;

  // robot model
  robot_model_.init_from_xml(robot_description_pinocchio_);
  object_model_.init_from_xml(object_description_);
}

double
OMAVInteractionCost::distance_from_obstacle_cost(const mppi::observation_t &x) {
  distance = std::sqrt(std::pow(x(0) - param_ptr_->x_obstacle, 2) +
                       std::pow(x(1) - param_ptr_->y_obstacle, 2));
  distance_from_savezone = (distance - (2 + 1));
  obstacle_cost =
      -param_ptr_->Q_obstacle * std::min(0.0, distance_from_savezone);
  return obstacle_cost;
}

mppi::CostBase::cost_t
OMAVInteractionCost::compute_cost(const mppi::observation_t &x,
                                  const mppi::reference_t &ref,
                                  const double t) {
  // Initialize Cost
  double cost = 0.0;

  // Get mode from reference vector
  mode = ref(8);
  // Update robot_model for kinematic calculations
  Eigen::VectorXd q_omav(7);
  q_omav << x.head<3>(), x.segment<3>(4), x(3);
  robot_model_.update_state(q_omav);
  // Update object_model for kinematic calculations
  Eigen::VectorXd q_object(1);
  q_object << x(13);
  object_model_.update_state(q_object);

  // Leafing Field Cost
  if (x(0) > param_ptr_->x_limit_max || x(1) > param_ptr_->y_limit_max ||
      x(2) > param_ptr_->z_limit_max || x(0) < param_ptr_->x_limit_min ||
      x(1) < param_ptr_->y_limit_min) {
    cost += param_ptr_->Q_leafing_field;
  }
  // Too close to the floor cost
  if ((x(2) - param_ptr_->floor_thresh) < 0) {
    cost += -param_ptr_->Q_floor / param_ptr_->floor_thresh * x(2) +
            param_ptr_->Q_floor;
  }
  if (mode == 0) {
    // Pose Cost
    mppi_pinocchio::Pose current_pose, reference_pose;
    current_pose.translation = x.head<3>();
    current_pose.rotation = {x(3), x(4), x(5), x(6)};
    reference_pose.translation = ref.head<3>();
    reference_pose.rotation = {ref(3), ref(4), ref(5), ref(6)};
    delta_pose = mppi_pinocchio::get_delta(current_pose, reference_pose);
    cost += delta_pose.transpose() * param_ptr_->Q_pose * delta_pose;
    // Unwanted contact cost
    if (param_ptr_->contact_bool) {
      cost += 100 * x(18);
    }
  }

  if (mode == 1) {
    // Calculate distance between hook and handle
    hook_handle_vector = robot_model_.get_pose(hook_frame_).translation -
                         object_model_.get_pose(handle_frame_).translation;
    distance_hook_handle =
        sqrt(hook_handle_vector.transpose() * hook_handle_vector);

    // Object Cost
    object_cost = param_ptr_->Q_object_x * (x(13) - ref(7)) * (x(13) - ref(7));
    cost += object_cost;
    // Handle Hook distance cost
    cost += std::max(
        0.0, param_ptr_->Q_handle_hook / (1 - param_ptr_->handle_hook_thresh) *
                 (distance_hook_handle - param_ptr_->handle_hook_thresh));
    // COM Handle vector
    com_hook_ = robot_model_.get_pose("omav").translation -
                robot_model_.get_pose("tip").translation;

    // Tip velocity cost
    tip_lin_velocity_ = x.segment<3>(7) + x.segment<3>(10).cross(com_hook_);
    tip_velocity_ << tip_lin_velocity_, x.segment<3>(10);

    cost += tip_velocity_.transpose() * param_ptr_->Q_vel * tip_velocity_;

    // Force Normed
    force_normed_ = x.segment<3>(15).normalized();
    // Torque Cost
    // torque_ = x.segment<3>(15).cross(com_hook_);
    torque_angle_ = force_normed_.dot(com_hook_.normalized());

    // cost += param_ptr_->Q_torque * torque_.transpose() * torque_;
    cost += std::min(param_ptr_->Q_torque,
                     param_ptr_->Q_torque * (1 - torque_angle_));
    // Efficiency cost
    double power_normed =
        x.segment<3>(15).normalized().dot(tip_lin_velocity_.normalized());
    cost +=
        std::min(param_ptr_->Q_power, param_ptr_->Q_power * (1 - power_normed));

    if (false) {
      std::cout << "------------- Velocity Cost -------------" << std::endl;
      std::cout << tip_velocity_.transpose() * param_ptr_->Q_vel * tip_velocity_
                << std::endl;
      std::cout << "------------- Handle Hook Cost -------------" << std::endl;
      std::cout << std::max(0.0, param_ptr_->Q_handle_hook /
                                     (1 - param_ptr_->handle_hook_thresh) *
                                     (distance_hook_handle -
                                      param_ptr_->handle_hook_thresh))
                << std::endl;
      std::cout << "------------- delta_pose_cost -------------" << std::endl;
      std::cout << object_cost << std::endl;
      std::cout << "------------- torque angle cost -------------" << std::endl;
      std::cout << std::min(param_ptr_->Q_torque,
                            param_ptr_->Q_torque * (1 - torque_angle_))
                << std::endl;
      std::cout << "------------- efficiency cost -------------" << std::endl;
      std::cout << std::min(param_ptr_->Q_power,
                            param_ptr_->Q_power * (1 - power_normed))
                << std::endl;
    }

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
  if (!nh.getParam("orientation_weight", Q_orientation) || Q_orientation < 0) {
    ROS_ERROR("Failed to parse orientation_cost or invalid!");
    return false;
  }
  if (!nh.getParam("x_object_weight", Q_object_x) || Q_object_x < 0) {
    ROS_ERROR("Failed to parse x_object_weight or invalid!");
    return false;
  }
  if (!nh.getParam("y_object_weight", Q_object_y) || Q_object_y < 0) {
    ROS_ERROR("Failed to parse y_object_cost or invalid!");
    return false;
  }
  if (!nh.getParam("z_object_weight", Q_object_z) || Q_object_z < 0) {
    ROS_ERROR("Failed to parse z_object_cost or invalid!");
    return false;
  }
  if (!nh.getParam("orientation_object_weight", Q_object_orientation) ||
      Q_object_orientation < 0) {
    ROS_ERROR("Failed to parse orientation_object_weight or invalid!");
    return false;
  }
  if (!nh.getParam("lin_vel_weight", Q_lin_vel) || Q_lin_vel < 0) {
    ROS_ERROR("Failed to parse lin_vel_weight or invalid!");
    return false;
  }
  if (!nh.getParam("ang_vel_weight", Q_ang_vel) || Q_ang_vel < 0) {
    ROS_ERROR("Failed to parse ang_vel_weight or invalid!");
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
  if (!nh.getParam("handle_hook_cost", Q_handle_hook)) {
    ROS_ERROR("Failed to parse handle_hook_cost or invalid!");
    return false;
  }
  if (!nh.getParam("handle_hook_threshold", handle_hook_thresh)) {
    ROS_ERROR("Failed to parse handle_hook_threshold or invalid!");
    return false;
  }
  if (!nh.getParam("power_cost", Q_power) || Q_power < 0) {
    ROS_ERROR("Failed to parse power_cost or invalid!");
    return false;
  }

  // Construction of the cost matrices
  pose_costs << Q_distance_x, Q_distance_y, Q_distance_z, Q_orientation,
      Q_orientation, Q_orientation;
  Q_pose = pose_costs.asDiagonal();

  object_costs << Q_object_x, Q_object_y, Q_object_z, Q_object_orientation,
      Q_object_orientation, Q_object_orientation;
  Q_object = object_costs.asDiagonal();

  vel_costs << Q_lin_vel, Q_lin_vel, Q_lin_vel, Q_ang_vel, Q_ang_vel, 0;
  Q_vel = vel_costs.asDiagonal();
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
