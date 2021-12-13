/*!
 * @file     cost.cpp
 * @author   Matthias Studiger
 * @date     10.04.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_omav_interaction/cost.h"

using namespace omav_interaction;

OMAVInteractionCost::OMAVInteractionCost(
    const std::string &robot_description_pinocchio,
    const std::string &object_description, OMAVInteractionCostParam *param)
    : robot_description_pinocchio_(robot_description_pinocchio),
      object_description_(object_description) {
  param_ptr_ = param;

  // robot model
  robot_model_.init_from_xml(robot_description_pinocchio_);
  object_model_.init_from_xml(object_description_);
}

double OMAVInteractionCost::distance_from_obstacle_cost(
    const mppi::observation_t &x) {
  distance = std::sqrt(std::pow(x(0) - param_ptr_->x_obstacle, 2) +
                       std::pow(x(1) - param_ptr_->y_obstacle, 2));
  distance_from_savezone = (distance - (2 + 1));
  obstacle_cost =
      -param_ptr_->Q_obstacle * std::min(0.0, distance_from_savezone);
  return obstacle_cost;
}

mppi::CostBase::cost_t OMAVInteractionCost::compute_cost(
    const mppi::observation_t &x, const mppi::reference_t &ref,
    const double t) {
  // Initialize Cost
  double cost = 0.0;
  cost_vector_.setZero();

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
  // Velocity Cost
  // compute_velocity_cost(x.segment<3>(26), x.segment<3>(29));
  // cost += cost_vector_(CostIdx::velocity);
  // leaving Field Cost
  if (x(0) > param_ptr_->x_limit_max || x(1) > param_ptr_->y_limit_max ||
      x(2) > param_ptr_->z_limit_max || x(0) < param_ptr_->x_limit_min ||
      x(1) < param_ptr_->y_limit_min) {
    cost += param_ptr_->Q_leaving_field;
  }
  // Too close to the floor cost
  compute_floor_cost(x(2));
  cost += cost_vector_(CostIdx::floor);

  if (mode == 0) {
    // Pose Cost
    compute_pose_cost(x, ref);
    cost += cost_vector_(CostIdx::pose);
    // Unwanted contact cost
    if (param_ptr_->contact_bool) {
      cost += 100 * x(18);
    }
  }

  if (mode == 1) {
    // Calculate all the necessary vectors
    compute_vectors();
    // Handle Hook Cost
    compute_handle_hook_cost();
    cost += cost_vector_(CostIdx::handle_hook);
    // Object Cost
    compute_object_cost(x, ref);
    cost += cost_vector_(CostIdx::object);

    // Torque Cost
    compute_torque_cost(x);
    cost += cost_vector_(CostIdx::torque);
    // Efficiency cost
    // compute_efficiency_cost(x);
    // cost += cost_vector_(CostIdx::efficiency);
    // Minimize force
    double force = x.segment<3>(15).norm();
    cost += param_ptr_->Q_power * force * force;
  }
  cost_ = cost;

  return cost;
}

void OMAVInteractionCost::compute_floor_cost(const double &omav_z) {
  if ((omav_z - param_ptr_->floor_thresh) < 0) {
    cost_vector_(CostIdx::floor) =
        -param_ptr_->Q_floor / param_ptr_->floor_thresh * omav_z +
        param_ptr_->Q_floor;
  }
}

void OMAVInteractionCost::compute_pose_cost(
    const Eigen::VectorXd &omav_state, const Eigen::VectorXd &omav_reference) {
  mppi_pinocchio::Pose current_pose, reference_pose;
  current_pose.translation = omav_state.head<3>();
  current_pose.rotation = {omav_state(3), omav_state(4), omav_state(5),
                           omav_state(6)};
  reference_pose.translation = omav_reference.head<3>();
  reference_pose.rotation = {omav_reference(3), omav_reference(4),
                             omav_reference(5), omav_reference(6)};
  delta_pose_ = mppi_pinocchio::get_delta(current_pose, reference_pose);
  cost_vector_(CostIdx::pose) =
      delta_pose_.transpose() * param_ptr_->Q_pose * delta_pose_;
}

void OMAVInteractionCost::compute_handle_hook_cost() {
  // Calculate distance between hook and handle
  distance_hook_handle_ =
      sqrt(hook_handle_vector_.transpose() * hook_handle_vector_);
  // Handle Hook distance cost
  cost_vector_(CostIdx::handle_hook) = std::max(
      0.0, param_ptr_->Q_handle_hook / (1 - param_ptr_->handle_hook_thresh) *
               (distance_hook_handle_ - param_ptr_->handle_hook_thresh));
}

void OMAVInteractionCost::compute_object_cost(
    const Eigen::VectorXd &omav_state, const Eigen::VectorXd &omav_reference) {
  cost_vector_(CostIdx::object) = param_ptr_->Q_object *
                                  (omav_state(13) - omav_reference(7)) *
                                  (omav_state(13) - omav_reference(7));
}

void OMAVInteractionCost::compute_vectors() {
  // Vector from hook to handle
  hook_handle_vector_ = robot_model_.get_pose(frames_.hook).translation -
                        object_model_.get_pose(frames_.handle_link).translation;
  // Vector from hook to omav
  com_hook_vector_ = robot_model_.get_pose(frames_.omav).translation -
                     robot_model_.get_pose(frames_.tip).translation;
  // Vector from handle ref to handle link eg. orthogonal to door panel
  handle_orthogonal_vector_ =
      object_model_.get_pose(frames_.handle_link).translation -
      object_model_.get_pose(frames_.handle_ref).translation;

  hook_pos_ = robot_model_.get_pose(frames_.hook).translation;
}

void OMAVInteractionCost::compute_tip_velocity_cost(
    const Eigen::VectorXd &omav_state) {
  // Tip velocity cost
  tip_lin_velocity_ = omav_state.segment<3>(7) +
                      omav_state.segment<3>(10).cross(com_hook_vector_);
  tip_velocity_ << tip_lin_velocity_, omav_state.segment<3>(10);

  cost_vector_(CostIdx::tip_velocity) =
      tip_velocity_.transpose() * param_ptr_->Q_vel * tip_velocity_;
}

void OMAVInteractionCost::compute_torque_cost(
    const Eigen::VectorXd &omav_state) {
  torque_angle_ =
      handle_orthogonal_vector_.normalized().dot(com_hook_vector_.normalized());

  cost_vector_(CostIdx::torque) = std::min(
      param_ptr_->Q_torque, param_ptr_->Q_torque * (1 - torque_angle_));
}

void OMAVInteractionCost::compute_efficiency_cost(
    const Eigen::VectorXd &omav_state) {
  double power_normed = handle_orthogonal_vector_.normalized().dot(
      tip_lin_velocity_.normalized());
  cost_vector_(CostIdx::efficiency) =
      std::min(param_ptr_->Q_power, param_ptr_->Q_power * (1 - power_normed));
}

void OMAVInteractionCost::compute_velocity_cost(
    const Eigen::Vector3d &linear_velocity,
    const Eigen::Vector3d &angular_velocity) {
  Eigen::Matrix<double, 6, 1> velocity_vector;
  velocity_vector << linear_velocity, angular_velocity;
  cost_vector_(CostIdx::velocity) =
      velocity_vector.transpose() * param_ptr_->Q_vel * velocity_vector;
}

bool OMAVInteractionCostParam::parse_from_ros(const ros::NodeHandle &nh) {
  getParameter(nh, "x_distance_weight", Q_distance_x, true);
  getParameter(nh, "y_distance_weight", Q_distance_y, true);
  getParameter(nh, "z_distance_weight", Q_distance_z, true);
  getParameter(nh, "orientation_weight", Q_orientation, true);
  getParameter(nh, "object_weight", Q_object, true);
  getParameter(nh, "lin_vel_weight", Q_lin_vel, true);
  getParameter(nh, "ang_vel_weight", Q_ang_vel, true);
  getParameter(nh, "leaving_field_cost", Q_leaving_field, true);
  getParameter(nh, "field_limit_x_min", x_limit_min, false);
  getParameter(nh, "field_limit_x_max", x_limit_max, false);
  getParameter(nh, "field_limit_y_min", y_limit_min, false);
  getParameter(nh, "field_limit_y_max", y_limit_max, false);
  getParameter(nh, "field_limit_z_max", z_limit_max, true);
  getParameter(nh, "floor_threshold", floor_thresh, true);
  getParameter(nh, "floor_cost", Q_floor, true);
  getParameter(nh, "obstacle_cost", Q_obstacle, true);
  getParameter(nh, "obstacle_position_x", x_obstacle, false);
  getParameter(nh, "obstacle_position_y", y_obstacle, false);
  getParameter(nh, "handle_hook_cost", Q_handle_hook, false);
  getParameter(nh, "handle_hook_threshold", handle_hook_thresh, false);
  getParameter(nh, "power_cost", Q_power, true);
  getParameter(nh, "torque_cost", Q_torque, true);

  // Construction of the cost matrices
  pose_costs << Q_distance_x, Q_distance_y, Q_distance_z, Q_orientation,
      Q_orientation, Q_orientation;
  Q_pose = pose_costs.asDiagonal();

  vel_costs << Q_lin_vel, Q_lin_vel, Q_lin_vel, Q_ang_vel, Q_ang_vel, Q_ang_vel;
  Q_vel = vel_costs.asDiagonal();
  return true;
}

template <class A>
bool OMAVInteractionCostParam::getParameter(const ros::NodeHandle &nh,
                                            const std::string &id, A &val,
                                            const bool &checkPositive) {
  if (!nh.getParam("torque_cost", val) || (checkPositive && val < 0)) {
    ROS_ERROR("Failed to parse %s or invalid!", id.c_str());
    return false;
  }
  return true;
}

std::ostream &operator<<(
    std::ostream &os, const omav_interaction::OMAVInteractionCostParam &param) {
  // clang-format off
    os << "========================================" << std::endl;
    os << "         OMAV Cost Parameters           " << std::endl;
    os << "========================================" << std::endl;
    os << " x_distance_weight: " << param.Q_distance_x << std::endl;
    os << " y_distance_weight: " << param.Q_distance_y << std::endl;
    os << " z_distance_weight: " << param.Q_distance_z << std::endl;
    os << " orientation_weight: " << param.Q_orientation << std::endl;
    os << " object_weight: " << param.Q_object << std::endl;
    os << " leaving_field_cost: " << param.Q_leaving_field << std::endl;
    os << " field_limit_x_min: " << param.x_limit_min << std::endl;
    os << " field_limit_x_max: " << param.x_limit_max << std::endl;
    os << " field_limit_y_min: " << param.y_limit_min << std::endl;
    os << " field_limit_y_max: " << param.y_limit_max << std::endl;
    os << " field_limit_z_max: " << param.z_limit_max << std::endl;
    os << " floor_threshold: " << param.floor_thresh << std::endl;
    os << " floor_cost: " << param.Q_floor << std::endl;
    os << " obstacle_weight: " << param.Q_obstacle << std::endl;
    os << " obstacle_position_x: " << param.x_obstacle << std::endl;
    os << " obstacle_position_y: " << param.y_obstacle << std::endl;
    os << " handle_hook_threshold: " << param.handle_hook_thresh << std::endl;
    os << " handle_hook_weight: " << param.Q_handle_hook << std::endl;
    os << " lin_vel_weight: " << param.Q_lin_vel << std::endl;
    os << " ang_vel_weight: " << param.Q_ang_vel << std::endl;
    os << " efficiency_weight: " << param.Q_power << std::endl;
    os << " torque_weight: " << param.Q_torque << std::endl;

    return os;
}
