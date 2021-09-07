//
// Created by giuseppe on 22.01.21.
//

#include <mppi_manipulation_royalpanda/controller.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <signal_logger/signal_logger.hpp>

#include <franka/robot_state.h>

#include <manipulation_msgs/conversions.h>

using namespace manipulation;
using namespace manipulation_royalpanda;

bool ManipulationController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& root_nh,
                                  ros::NodeHandle& controller_nh) {
  if (!init_parameters(controller_nh)) return false;
  if (!init_interfaces(robot_hw)) return false;
  init_ros(controller_nh);

  man_interface_ = std::make_unique<PandaControllerInterface>(controller_nh);
  if (!man_interface_->init()) {
    ROS_ERROR("Failed to initialized manipulation interface");
    return false;
  }
  ROS_INFO("Solver initialized correctly.");

  started_ = false;
  state_ok_ = true;
  ROS_INFO("Controller successfully initialized!");
  return true;
}

bool ManipulationController::init_parameters(ros::NodeHandle& node_handle) {
  if (!node_handle.getParam("arm_id", arm_id_)) {
    ROS_ERROR("Could not read parameter arm_id");
    return false;
  }

  if (!node_handle.getParam("joint_names", joint_names_) ||
      joint_names_.size() != 7) {
    ROS_ERROR("Invalid or no joint_names parameters provided");
    return false;
  }

  if (!node_handle.getParam("base_joint_names", base_joint_names_) ||
      base_joint_names_.size() != 3) {
    ROS_WARN(
        "No base_joint_names parameters provided. These work only in "
        "simulation.");
  }

  if (!node_handle.getParam("sequential", sequential_)) {
    ROS_ERROR("Failed to get sequential parameter");
    return false;
  }

  if (!gains_.init_from_ros(node_handle, "controller_")) {
    ROS_ERROR("Failed to parse gains.");
    return false;
  }

  if (!node_handle.getParam("state_topic", state_topic_)) {
    ROS_ERROR("state_topic not found");
    return false;
  }

  if (!node_handle.getParam("nominal_state_topic", nominal_state_topic_)) {
    ROS_ERROR("nominal_state_topic not found");
    return false;
  }

  if (!node_handle.getParam("base_twist_topic", base_twist_topic_)) {
    ROS_ERROR("base_twist_topic not found");
    return false;
  }

  if (!node_handle.getParam("base_twist_topic", base_twist_topic_)) {
    ROS_ERROR("base_twist_topic not found");
    return false;
  }

  if (!node_handle.getParam("apply_filter", apply_filter_)) {
    ROS_ERROR("apply_filter not found");
    return false;
  }

  if (!safety_filter_params_.init_from_ros(node_handle)) {
    ROS_ERROR("Failed to parse safety filter parameters.");
    return false;
  }

  ROS_INFO("Parameters successfully initialized.");
  return true;
}

bool ManipulationController::init_interfaces(
    hardware_interface::RobotHW* robot_hw) {
  auto* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR("Failed to get effort joint interface.");
    return false;
  }

  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(
          effort_joint_interface->getHandle(joint_names_[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR("Failed to get joint handles: ", ex.what());
      return false;
    }
  }

  auto* velocity_joint_interface =
      robot_hw->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface == nullptr) {
    ROS_ERROR("Failed to get velocity joint interface.");
    return false;
  }

  for (size_t i = 0; i < 3; ++i) {
    try {
      base_joint_handles_.push_back(
          velocity_joint_interface->getHandle(base_joint_names_[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_WARN_STREAM("Failed to get base joint handles: "
                      << base_joint_names_[i]
                      << ". These will only work in simulation --> sending "
                         "command via ros topic instead.");
      has_base_handles_ = false;
      break;
    }
  }
  has_base_handles_ = true;

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR("Failed to get model interface.");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id_ + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR("Failed to get model handle from interface: ", ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("Failed to get state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id_ + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR("Failed to get state handle from interface: ", ex.what());
    return false;
  }

  ROS_INFO("Interfaces successfully initialized.");
  return true;
}

void ManipulationController::init_ros(ros::NodeHandle& nh) {
  base_twist_publisher_.init(nh, base_twist_topic_, 1);
  nominal_state_publisher_.init(nh, nominal_state_topic_, 1);
  state_subscriber_ = nh.subscribe(
      state_topic_, 1, &ManipulationController::state_callback, this);

  ROS_INFO_STREAM("Sending base commands to: " << base_twist_topic_);
  ROS_INFO_STREAM("Receiving state at: " << state_topic_);
  ROS_INFO_STREAM("Ros successfully initialized.");
}

void ManipulationController::state_callback(
    const manipulation_msgs::StateConstPtr& state_msg) {
  if (!started_) return;

  {
    std::unique_lock<std::mutex> lock(observation_mutex_);
    manipulation::conversions::msgToEigen(*state_msg, x_, observation_time_);

    if (!state_received_) {
      position_desired_[0] = state_msg->base_pose.x;
      position_desired_[1] = state_msg->base_pose.y;
      position_desired_[2] = state_msg->base_pose.z;
      position_initial_.head<3>() = position_desired_.head<3>();
      if (!man_interface_->init_reference_to_current_pose(x_,
                                                          observation_time_)) {
        ROS_WARN("Failed to set the controller reference to current state.");
      }
    }
  }

  x_ros_ = *state_msg;
  state_received_ = true;
}

void ManipulationController::starting(const ros::Time& time) {
  if (started_) return;

  // with sequential execution the optimization needs to be explicitly called
  // in the update function of the controller (not real-time safe)
  if (!sequential_) {
    started_ = man_interface_->start();
    if (!started_) {
      ROS_ERROR("Failed  to start controller");
      return;
    }
  }

  // we do not actively control the gripper
  u_opt_.setZero(10);
  velocity_measured_.setZero(10);
  velocity_filtered_.setZero(10);
  position_desired_.setZero(10);
  position_measured_.setZero(10);
  position_initial_.setZero(10);
  for (size_t i = 0; i < 7; i++) {
    position_desired_[i + 3] = joint_handles_[i].getPosition();
    position_initial_[i + 3] = joint_handles_[i].getPosition();
  }
  position_measured_ = position_desired_;

  // reset the energy tank initial state
  double initial_tank_state =
      std::sqrt(2.0 * safety_filter_params_.initial_tank_energy);
  energy_tank_.reset(initial_tank_state, time.toSec());

  // reset the safety filter
  safety_filter_ =
      std::make_unique<PandaMobileSafetyFilter>(safety_filter_params_);

  // metrics
  stage_cost_ = 0.0;

  // logging
  signal_logger::logger->stopLogger();
  signal_logger::add(u_opt_, "velocity_command");
  signal_logger::add(arm_torque_command_, "torque_command");
  signal_logger::add(velocity_measured_, "velocity_measured");
  signal_logger::add(velocity_filtered_, "velocity_filtered");
  signal_logger::add(position_measured_, "position_measured");
  signal_logger::add(position_desired_, "position_desired");
  signal_logger::add(stage_cost_, "stage_cost");
  signal_logger::add(power_channels_, "power_channels");
  signal_logger::add(power_from_error_, "power_from_error");
  signal_logger::add(power_from_interaction_, "power_from_interaction");
  signal_logger::add(total_power_exchange_, "total_power_exchange");
  signal_logger::add(external_torque_, "external_torque");
  signal_logger::add(energy_tank_.get_state(), "tank_state");
  for (const auto& constraint : safety_filter_->constraints_) {
    signal_logger::add(constraint.second->violation_,
                       constraint.first + "_violation");
  }
  signal_logger::logger->startLogger(true);

  started_ = true;
  ROS_INFO("Controller started!");
}

void ManipulationController::enforce_constraints(const ros::Duration& period) {
  {
    std::unique_lock<std::mutex> lock(observation_mutex_);
    x_(STATE_DIMENSION - TORQUE_DIMENSION - 1) = energy_tank_.get_state();
    safety_filter_->update(x_, u_, observation_time_);

    // this is required for computing some metrics
    // we provide only the joints position (no gripper) since the implementation
    // of the joint limits in the safety_filter package assumes that the state
    // vector is eventually only the joint state
    // TODO(giuseppe) each problem shuould have its implementation (maybe
    // inherithed)
    //   as this is not working anymore if another constraint requires the full
    //   state instead.
    safety_filter_->update_violation(x_.head<10>());
  }

  if (apply_filter_) {
    safety_filter_->apply(u_opt_);
  } else {
    u_opt_ = u_.head<10>();  // safety filter does not account for the gripper
  }

  // compute the total power exchange with the tank
  external_torque_ = x_.tail<TORQUE_DIMENSION>().head<10>();
  power_channels_ = -u_opt_.cwiseProduct(external_torque_);
  power_from_error_ = (u_opt_ - velocity_filtered_).transpose() *
                      (position_desired_ - position_initial_);
  power_from_interaction_ = power_channels_.sum();
  total_power_exchange_ = power_from_error_ + power_from_interaction_;
  energy_tank_.step(total_power_exchange_, period.toSec());
}

void ManipulationController::send_command_base(const ros::Duration& period) {
  // In simulation, we can use directly the hardware interface
  if (has_base_handles_) {
    for (int i = 0; i < 3; i++) {
      base_joint_handles_[i].setCommand(u_[i]);
    }
    return;
  }

  if (base_trigger_() && base_twist_publisher_.trylock()) {
    Eigen::Vector3d twist_nominal(x_nom_ros_.base_twist.linear.x,
                                  x_nom_ros_.base_twist.linear.y,
                                  x_nom_ros_.base_twist.angular.z);

    base_twist_publisher_.msg_.linear.x = u_opt_[0];
    base_twist_publisher_.msg_.linear.y = u_opt_[1];
    base_twist_publisher_.msg_.angular.z = u_opt_[2];
    base_twist_publisher_.unlockAndPublish();
  }
}

void ManipulationController::send_command_arm(const ros::Duration& period) {
  // clang-format off
  std::array<double, 7> tau_d_calculated{};
  for (int i = 0; i < 7; ++i) {
    tau_d_calculated[i] =
        std::min(gains_.arm_gains.Imax[i], std::max(-gains_.arm_gains.Imax[i], gains_.arm_gains.Ki[i] * (position_desired_[3+i] - robot_state_.q[i])))
        - gains_.arm_gains.Kd[i] * velocity_filtered_[3+i];
  }

  // max torque diff with sampling rate of 1 kHz is 1000 * (1 / sampling_time).
  saturateTorqueRate(tau_d_calculated, robot_state_.tau_J_d, arm_torque_command_);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(arm_torque_command_[i]);
  }
  // clang-format on
}

void ManipulationController::update(const ros::Time& time,
                                    const ros::Duration& period) {
  if (!started_) {
    ROS_ERROR("Controller not started. Probably error occurred...");
    return;
  }

  if (!state_received_) {
    ROS_WARN_STREAM_THROTTLE(
        2.0, "[ManipulationController::update] State not received yet.");
    return;
  }

  if (!state_ok_) {
    ROS_WARN_STREAM_THROTTLE(
        2.0, "[ManipulationController::update] State is not ok.");
    return;
  }

  {
    std::unique_lock<std::mutex> lock(observation_mutex_);
    man_interface_->set_observation(x_, time.toSec());
  }
  man_interface_->update_reference(time.toSec());

  ROS_DEBUG_STREAM("Ctl state:"
                   << std::endl
                   << std::setprecision(2)
                   << manipulation::conversions::eigenToString(x_));

  if (sequential_) {
    man_interface_->update_policy();
    //    man_interface_->get_input(x_, u_, time.toSec());
    man_interface_->get_input_state(x_, x_nom_, u_, time.toSec());
    man_interface_->publish_ros_default();
    man_interface_->publish_ros();
  } else {
    man_interface_->get_input_state(x_, x_nom_, u_, time.toSec());
  }

  manipulation::conversions::eigenToMsg(x_nom_, time.toSec(), x_nom_ros_);
  robot_state_ = state_handle_->getRobotState();

  static const double alpha = 0.1;
  position_measured_.head<3>() = x_.head<3>();
  velocity_measured_.head<3>() = x_.segment<3>(12);
  velocity_filtered_.head<3>() = (1 - alpha) * velocity_filtered_.head<3>() +
                                 alpha * velocity_measured_.head<3>();
  for (int i = 0; i < 7; i++) {
    position_measured_[i + 3] = robot_state_.q[i];
    velocity_measured_[i + 3] = robot_state_.dq[i];
    velocity_filtered_[i + 3] =
        (1 - alpha) * velocity_filtered_[i + 3] + alpha * robot_state_.dq[i];
  }

  getRotationMatrix(R_world_base, x_(2));
  position_desired_.tail<7>() += u_opt_.tail<7>() * period.toSec();
  position_desired_.head<3>() +=
      R_world_base * u_opt_.head<3>() * period.toSec();

  enforce_constraints(period);
  send_command_arm(period);
  send_command_base(period);

  if (nominal_state_publisher_.trylock()) {
    nominal_state_publisher_.msg_ = x_nom_ros_;
    nominal_state_publisher_.unlockAndPublish();
  }

  {
    std::unique_lock<std::mutex> lock(observation_mutex_);
    stage_cost_ = man_interface_->get_stage_cost(x_, u_opt_, time.toSec());
  }
  signal_logger::logger->collectLoggerData();
}

void ManipulationController::stopping(const ros::Time& time) {
  signal_logger::logger->disableElement("/log/torque_command");
  signal_logger::logger->disableElement("/log/stage_cost");
  for (const auto& constraint : safety_filter_->constraints_) {
    signal_logger::logger->disableElement("/log/" + constraint.first +
                                          "_violation");
  }
}

void ManipulationController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d,
    Eigen::Matrix<double, 7, 1>&
        torque_cmd) {  // NOLINT (readability-identifier-naming)
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    torque_cmd[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_),
                                          -delta_tau_max_);
  }
}

void ManipulationController::getRotationMatrix(Eigen::Matrix3d& R,
                                               const double theta) {
  // clang-format off
  R << std::cos(theta), std::sin(theta), 0.0,
      -std::sin(theta), std::cos(theta), 0.0,
       0.0, 0.0, 1.0;
  // clang-format on
}

PLUGINLIB_EXPORT_CLASS(manipulation_royalpanda::ManipulationController,
                       controller_interface::ControllerBase)