/*!
 * @file     cost.cpp
 * @author   Maximilian Brunner
 * @date     25.11.2021
 * @version  1.0
 * @brief    description
 */

#include "mppi_omav_interaction/cost_valve.h"

using namespace omav_interaction;

OMAVInteractionCostValve::OMAVInteractionCostValve(
    const std::string &robot_description_pinocchio,
    const std::string &object_description, OMAVInteractionCostValveParam *param)
    : robot_description_pinocchio_(robot_description_pinocchio),
      object_description_(object_description),
      param_ptr_(param) {
  robot_model_.init_from_xml(robot_description_pinocchio_);
  object_model_.init_from_xml(object_description_);
  cost_vector_.resize(cost_description::SIZE_COST_VECTOR);
}

mppi::CostBase::cost_t OMAVInteractionCostValve::compute_cost(
    const mppi::observation_t &x, const mppi::reference_t &ref,
    const double t) {
  cost_vector_.setZero();

  // Get mode from reference vector
  mode_ = ref(reference_description::INTERACTION_MODE);

  // Update robot_model for kinematic calculations
  Eigen::VectorXd q_omav(7);
  q_omav << x.segment<3>(
      omav_state_description_simulation::MAV_POSITION_X_WORLD),
      x.segment<3>(omav_state_description_simulation::MAV_ORIENTATION_X_WORLD),
      x(omav_state_description_simulation::MAV_ORIENTATION_W_WORLD);
  robot_model_.update_state(q_omav);

  // Update object_model for kinematic calculations
  Eigen::VectorXd q_object(1);
  q_object << x(omav_state_description_simulation::OBJECT_HINGE_ORIENTATION);
  object_model_.update_state(q_object);
  compute_vectors();

  // Compute cost on *reference* velocity, not on actual velocity
  compute_velocity_cost(x.segment<3>(omav_state_description_simulation::
                                         MAV_LINEAR_VELOCITY_X_DESIRED_WORLD),
                        x.segment<3>(omav_state_description_simulation::
                                         MAV_ANGULAR_VELOCITY_X_DESIRED_BODY));

  if (mode_ == interaction_mode::FREE_FLIGHT) {
    compute_pose_cost(x, ref);

    // Unwanted contact cost
    if (param_ptr_->contact_bool) {
      cost_vector_(cost_description::CONTACT_COST) =
          100 * x(omav_state_description_simulation::CONTACT_STATE);
    }
  } else if (mode_ == interaction_mode::INTERACTION) {
    compute_pose_cost(x, ref);
    compute_handle_hook_cost();
    compute_object_cost(x, ref);

    Eigen::VectorXd interaction_force =
        x.segment<3>(omav_state_description_simulation::INTERACTION_FORCE_X);
    cost_vector_(cost_description::FORCE_COST) =
        (interaction_force.cwiseProduct(param_ptr_->Q_forcev)
             .cwiseProduct(interaction_force))
            .sum();
  }

  cost_ = cost_vector_.sum();

  return cost_;
}

void OMAVInteractionCostValve::compute_pose_cost(
    const Eigen::VectorXd &omav_state, const Eigen::VectorXd &omav_reference) {
  mppi_pinocchio::Pose current_pose;
  mppi_pinocchio::Pose reference_pose;

  current_pose.translation = omav_state.segment<3>(
      omav_state_description_simulation::MAV_POSITION_X_WORLD);
  current_pose.rotation = {
      omav_state(omav_state_description_simulation::MAV_ORIENTATION_W_WORLD),
      omav_state(omav_state_description_simulation::MAV_ORIENTATION_X_WORLD),
      omav_state(omav_state_description_simulation::MAV_ORIENTATION_Y_WORLD),
      omav_state(omav_state_description_simulation::MAV_ORIENTATION_Z_WORLD)};

  reference_pose.translation = omav_reference.segment<3>(
      reference_description::MAV_GOAL_POSITION_X_WORLD);
  reference_pose.rotation = {
      omav_reference(reference_description::MAV_GOAL_ORIENTATION_W_WORLD),
      omav_reference(reference_description::MAV_GOAL_ORIENTATION_X_WORLD),
      omav_reference(reference_description::MAV_GOAL_ORIENTATION_Y_WORLD),
      omav_reference(reference_description::MAV_GOAL_ORIENTATION_Z_WORLD)};

  delta_pose_ = mppi_pinocchio::get_delta(current_pose, reference_pose);

  if (mode_ == interaction_mode::FREE_FLIGHT) {
    cost_vector_(cost_description::POSE_COST) =
        delta_pose_.transpose() * param_ptr_->Q_pose * delta_pose_;
  } else if (mode_ == interaction_mode::INTERACTION) {
    cost_vector_(cost_description::POSE_COST) =
        delta_pose_.transpose() * param_ptr_->Q_pose_int * delta_pose_;
  }
}

void OMAVInteractionCostValve::compute_handle_hook_cost() {
  // Calculate distance between hook and handle
  distance_hook_handle_ = hook_handle_vector_.norm();
  if (distance_hook_handle_ > param_ptr_->handle_hook_thresh) {
    cost_vector_(cost_description::HANDLE_HOOK_COST) =
        param_ptr_->Q_handle_hook *
        (distance_hook_handle_ - param_ptr_->handle_hook_thresh);
  }
  cost_vector_(cost_description::HANDLE_HOOK_COST) +=
      hook_handle_vector_(2) * hook_handle_vector_(2) *
      param_ptr_->Q_handle_hookV;
}

void OMAVInteractionCostValve::compute_object_cost(
    const Eigen::VectorXd &omav_state, const Eigen::VectorXd &omav_reference) {
  double object_orientation_error =
      omav_state(omav_state_description_simulation::OBJECT_HINGE_ORIENTATION) -
      omav_reference(reference_description::OBJECT_GOAL_ORIENTATION);
  cost_vector_(cost_description::OBJECT_COST) =
      param_ptr_->Q_object * abs(object_orientation_error);
}

void OMAVInteractionCostValve::compute_vectors() {
  // Vector from hook to handle
  hook_handle_vector_ = robot_model_.get_pose(frames_.hook).translation -
                        object_model_.get_pose(frames_.handle_link).translation;
  r_omav_handle_I_ = robot_model_.get_pose(frames_.handle_link).translation -
                     object_model_.get_pose(frames_.omav).translation;
  // Vector from hook to omav
  r_tip_omav_I_ = robot_model_.get_pose(frames_.omav).translation -
                  robot_model_.get_pose(frames_.tip).translation;

  hook_pos_ = robot_model_.get_pose(frames_.hook).translation;
}

void OMAVInteractionCostValve::compute_velocity_cost(
    const Eigen::Vector3d &linear_velocity,
    const Eigen::Vector3d &angular_velocity) {
  Eigen::Matrix<double, 6, 1> velocity_vector;
  velocity_vector << linear_velocity, angular_velocity;
  cost_vector_(cost_description::VELOCITY_COST) =
      velocity_vector.transpose() * param_ptr_->Q_vel * velocity_vector;
}

bool OMAVInteractionCostValveParam::parse_from_ros(
    const ros::NodeHandle &private_nh) {
  ros::NodeHandle nh = ros::NodeHandle(private_nh, "cost_valve_parameters");
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
  suc = suc && getParameter(nh, "Q_unwanted_contact", Q_unwanted_contact, true);
  suc =
      suc && getParameter(nh, "handle_hook_thresh", handle_hook_thresh, false);
  suc = suc && getParameter(nh, "force_cost", Q_force, true);
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
bool OMAVInteractionCostValveParam::getParameter(const ros::NodeHandle &nh,
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
    const omav_interaction::OMAVInteractionCostValveParam &param) {
  // clang-format off
    os << "========================================" << std::endl;
    os << "         OMAV Cost Parameters           " << std::endl;
    os << "========================================" << std::endl;
    os << " Q_x_omav: " << param.Q_x_omav << std::endl;
    os << " Q_y_omav: " << param.Q_y_omav << std::endl;
    os << " Q_z_omav: " << param.Q_z_omav << std::endl;
    os << " orientation_weight: " << param.Q_orientation << std::endl;
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
    os << " contact_bool: " << param.contact_bool << std::endl;
    os << " unwanted_contact: " << param.Q_unwanted_contact << std::endl;

    return os;
}
