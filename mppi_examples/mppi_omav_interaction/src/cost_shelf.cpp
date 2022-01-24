/*!
 * @file     cost.cpp
 * @author   Matthias Studiger
 * @date     10.04.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_omav_interaction/cost_shelf.h"

using namespace omav_interaction;

OMAVInteractionCostShelf::OMAVInteractionCostShelf(
    const std::string &robot_description_pinocchio,
    const std::string &object_description, OMAVInteractionCostShelfParam *param)
    : robot_description_pinocchio_(robot_description_pinocchio),
      object_description_(object_description) {
  param_ptr_ = param;

  // robot model
  robot_model_.init_from_xml(robot_description_pinocchio_);
  object_model_.init_from_xml(object_description_);
}

// double OMAVInteractionCostShelf::distance_from_obstacle_cost(
//     const mppi::observation_t &x) {
//   distance = std::sqrt(std::pow(x(0) - param_ptr_->x_obstacle, 2) +
//                        std::pow(x(1) - param_ptr_->y_obstacle, 2));
//   distance_from_savezone = (distance - (2 + 1));
//   obstacle_cost =
//       -param_ptr_->Q_obstacle * std::min(0.0, distance_from_savezone);
//   return obstacle_cost;
// }

mppi::CostBase::cost_t OMAVInteractionCostShelf::compute_cost(
    const mppi::observation_t &x, const mppi::reference_t &ref,
    const double t) {
  // Initialize Cost
  // double cost = 0.0;
  cost_vector_.setZero();

  // Get mode from reference vector
  mode_ = ref(8);
  // Update robot_model for kinematic calculations
  Eigen::VectorXd q_omav(7);
  q_omav << x.head<3>(), x.segment<3>(4), x(3);
  robot_model_.update_state(q_omav);
  // Update object_model for kinematic calculations
  Eigen::VectorXd q_object(1);
  q_object << x(13);
  object_model_.update_state(q_object);

  // Compute costs:
  compute_velocity_cost(x.segment<3>(26), x.segment<3>(29));
  compute_floor_cost(x(2));
  compute_field_cost(x);
  compute_pose_cost(x, ref);

  if (mode_ == 0) {
    // Unwanted contact cost
    if (param_ptr_->contact_bool) {
      cost_vector_(CostIdx::contact) = 100.0 * x(18);
    }
  }

  if (mode_ == 1) {
    // Calculate all the necessary vectors
    compute_vectors();
    compute_handle_hook_cost();
    compute_object_cost(x, ref);
    compute_torque_cost(x);
    compute_efficiency_cost(x);
    double force = x.segment<3>(15).norm();
    cost_vector_(CostIdx::force) = param_ptr_->Q_force * force * force;
  }

  double cost = cost_vector_.sum();
  cost_ = cost;

  return cost;
}

void OMAVInteractionCostShelf::compute_field_cost(
    const mppi::observation_t &x) {
  if (x(0) > param_ptr_->x_limit_max || x(1) > param_ptr_->y_limit_max ||
      x(2) > param_ptr_->z_limit_max || x(0) < param_ptr_->x_limit_min ||
      x(1) < param_ptr_->y_limit_min) {
    cost_vector_(CostIdx::leaving_field) = param_ptr_->Q_leaving_field;
  }
}

void OMAVInteractionCostShelf::compute_floor_cost(const double &omav_z) {
  if ((omav_z - param_ptr_->floor_thresh) < 0) {
    cost_vector_(CostIdx::floor) =
        -param_ptr_->Q_floor / param_ptr_->floor_thresh * omav_z +
        param_ptr_->Q_floor;
  }
}

void OMAVInteractionCostShelf::compute_pose_cost(
    const Eigen::VectorXd &omav_state, const Eigen::VectorXd &omav_reference) {
  mppi_pinocchio::Pose current_pose, reference_pose;
  current_pose.translation = omav_state.head<3>();
  current_pose.rotation = {omav_state(3), omav_state(4), omav_state(5),
                           omav_state(6)};
  reference_pose.translation = omav_reference.head<3>();
  reference_pose.rotation = {omav_reference(3), omav_reference(4),
                             omav_reference(5), omav_reference(6)};
  delta_pose_ = mppi_pinocchio::get_delta(current_pose, reference_pose);
  if (mode_ == 0) {
    cost_vector_(CostIdx::pose) =
        delta_pose_.transpose() * param_ptr_->Q_pose * delta_pose_;
  } else {
    cost_vector_(CostIdx::pose) =
        delta_pose_.transpose() * param_ptr_->Q_pose_int * delta_pose_;
  }
}

void OMAVInteractionCostShelf::compute_handle_hook_cost() {
  // Calculate distance between hook and handle
  distance_hook_handle_ =
      sqrt(hook_handle_vector_.transpose() * hook_handle_vector_);
  // Handle Hook distance cost
  cost_vector_(CostIdx::handle_hook) = std::max(
      0.0, param_ptr_->Q_handle_hook / (1.0 - param_ptr_->handle_hook_thresh) *
               (distance_hook_handle_ - param_ptr_->handle_hook_thresh));
}

void OMAVInteractionCostShelf::compute_object_cost(
    const Eigen::VectorXd &omav_state, const Eigen::VectorXd &omav_reference) {
  cost_vector_(CostIdx::object) = param_ptr_->Q_object *
                                  (omav_state(13) - omav_reference(7)) *
                                  (omav_state(13) - omav_reference(7));
}

void OMAVInteractionCostShelf::compute_vectors() {
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

void OMAVInteractionCostShelf::compute_tip_velocity_cost(
    const Eigen::VectorXd &omav_state) {
  // Tip velocity cost
  tip_lin_velocity_ = omav_state.segment<3>(7) +
                      omav_state.segment<3>(10).cross(com_hook_vector_);
  tip_velocity_ << tip_lin_velocity_, omav_state.segment<3>(10);

  cost_vector_(CostIdx::tip_velocity) =
      tip_velocity_.transpose() * param_ptr_->Q_vel * tip_velocity_;
}

/**
 * @brief      Calculates the torque cost. This is based only on the angle
 *             between a normal vector on the door and the arm. The larger the
 *             angle, the larger the cost. The cost is limited to a range of
 *             [0,Q_torque].
 *
 * @param[in]  omav_state  The omav state
 */
void OMAVInteractionCostShelf::compute_torque_cost(
    const Eigen::VectorXd &omav_state) {
  // Compute cosine of the torque angle (dot product):
  torque_angle_ =
      handle_orthogonal_vector_.normalized().dot(com_hook_vector_.normalized());

  cost_vector_(CostIdx::torque) = std::min(
      param_ptr_->Q_torque, param_ptr_->Q_torque * (1.0 - torque_angle_));
}

/**
 * @brief      Calculates the efficiency cost. This is based on the alignment of
 *             the door handle with the velocity of the tip. The more parallel,
 *             the lower the cost.
 *
 * @param[in]  omav_state  The omav state
 */
void OMAVInteractionCostShelf::compute_efficiency_cost(
    const Eigen::VectorXd &omav_state) {
  double handle_tip_alignment = handle_orthogonal_vector_.normalized().dot(
      tip_lin_velocity_.normalized());
  cost_vector_(CostIdx::efficiency) =
      std::min(param_ptr_->Q_efficiency,
               param_ptr_->Q_efficiency * (1.0 - handle_tip_alignment));
}

void OMAVInteractionCostShelf::compute_velocity_cost(
    const Eigen::Vector3d &linear_velocity,
    const Eigen::Vector3d &angular_velocity) {
  Eigen::Matrix<double, 6, 1> velocity_vector;
  velocity_vector << linear_velocity, angular_velocity;
  cost_vector_(CostIdx::velocity) =
      velocity_vector.transpose() * param_ptr_->Q_vel * velocity_vector;
}

bool OMAVInteractionCostShelfParam::parse_from_ros(
    const ros::NodeHandle &private_nh) {
  ros::NodeHandle nh = ros::NodeHandle(private_nh, "cost_shelf_parameters");
  bool suc = true;
  suc = suc && getParameter(nh, "Q_x_omav", Q_x_omav, true);
  suc = suc && getParameter(nh, "Q_y_omav", Q_y_omav, true);
  suc = suc && getParameter(nh, "Q_z_omav", Q_z_omav, true);
  suc = suc && getParameter(nh, "Q_orientation_omav", Q_orientation, true);
  suc = suc && getParameter(nh, "Q_object", Q_object, true);
  suc = suc && getParameter(nh, "Q_linear_velocity", Q_lin_vel, true);
  double Q_roll, Q_pitch, Q_yaw;
  suc = suc && getParameter(nh, "Q_roll", Q_roll, true);
  suc = suc && getParameter(nh, "Q_pitch", Q_pitch, true);
  suc = suc && getParameter(nh, "Q_yaw", Q_yaw, true);
  double Q_int_roll, Q_int_pitch;
  suc = suc && getParameter(nh, "Q_int_roll", Q_int_roll, true);
  suc = suc && getParameter(nh, "Q_int_pitch", Q_int_pitch, true);
  suc = suc && getParameter(nh, "leaving_field_cost", Q_leaving_field, true);
  suc = suc && getParameter(nh, "field_limit_x_min", x_limit_min, false);
  suc = suc && getParameter(nh, "field_limit_x_max", x_limit_max, false);
  suc = suc && getParameter(nh, "field_limit_y_min", y_limit_min, false);
  suc = suc && getParameter(nh, "field_limit_y_max", y_limit_max, false);
  suc = suc && getParameter(nh, "field_limit_z_max", z_limit_max, true);
  suc = suc && getParameter(nh, "floor_thresh", floor_thresh, true);
  suc = suc && getParameter(nh, "floor_cost", Q_floor, true);
  suc = suc && getParameter(nh, "obstacle_cost", Q_obstacle, true);
  suc = suc && getParameter(nh, "obstacle_position_x", x_obstacle, false);
  suc = suc && getParameter(nh, "obstacle_position_y", y_obstacle, false);
  suc = suc && getParameter(nh, "handle_hook_cost", Q_handle_hook, true);
  suc =
      suc && getParameter(nh, "handle_hook_thresh", handle_hook_thresh, false);
  suc = suc && getParameter(nh, "efficiency_cost", Q_efficiency, true);
  suc = suc && getParameter(nh, "force_cost", Q_force, true);
  suc = suc && getParameter(nh, "torque_cost", Q_torque, true);
  suc = suc && getParameter(nh, "contact_prohibitor", contact_bool, false);

  // Construction of the cost matrices
  pose_costs << Q_x_omav, Q_y_omav, Q_z_omav, Q_orientation, Q_orientation,
      Q_orientation;
  pose_costs_int << 0, 0, 0, Q_int_roll, Q_int_pitch, 0;
  Q_pose = pose_costs.asDiagonal();
  Q_pose_int = pose_costs_int.asDiagonal();

  vel_costs << Q_lin_vel, Q_lin_vel, Q_lin_vel, Q_roll, Q_pitch, Q_yaw;
  Q_vel = vel_costs.asDiagonal();
  if (suc) {
    return true;
  }
  return false;
}

template <class A>
bool OMAVInteractionCostShelfParam::getParameter(const ros::NodeHandle &nh,
                                                 const std::string &id, A &val,
                                                 const bool &checkPositive) {
  if (!nh.getParam(id, val) || (checkPositive && val < 0)) {
    ROS_ERROR("Failed to parse %s or invalid!", id.c_str());
    return false;
  }
  return true;
}

std::ostream &operator<<(
    std::ostream &os,
    const omav_interaction::OMAVInteractionCostShelfParam &param) {
  // clang-format off
    os << "========================================" << std::endl;
    os << "         OMAV Cost Parameters           " << std::endl;
    os << "========================================" << std::endl;
    os << " Q_x_omav: " << param.Q_x_omav << std::endl;
    os << " Q_y_omav: " << param.Q_y_omav << std::endl;
    os << " Q_z_omav: " << param.Q_z_omav << std::endl;
    os << " Q_orientation_omav: " << param.Q_orientation << std::endl;
    os << " Q_object: " << param.Q_object << std::endl;
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
    os << " Q_linear_velocity: " << param.Q_lin_vel << std::endl;
    os << " ang_vel_weight: " << param.Q_ang_vel << std::endl;
    os << " efficiency_weight: " << param.Q_efficiency << std::endl;
    os << " torque_weight: " << param.Q_torque << std::endl;
    os << " force_weight: " << param.Q_force << std::endl;
    os << " contact_bool: " << param.contact_bool << std::endl;

    return os;
}
