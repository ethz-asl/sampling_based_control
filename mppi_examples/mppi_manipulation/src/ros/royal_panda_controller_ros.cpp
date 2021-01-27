//
// Created by giuseppe on 22.01.21.
//

#include <mppi_manipulation/ros/royal_panda_controller_ros.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace manipulation {

bool RoyalPandaControllerRos::init(hardware_interface::RobotHW* robot_hw,
                                   ros::NodeHandle& node_handle) {
  if (!init_parameters(node_handle)) return false;
  if (!init_interfaces(robot_hw)) return false;
  if (!init_publishers(node_handle)) return false;

  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);

  man_interface_ = std::make_unique<manipulation::PandaControllerInterface>(node_handle);
  if (!man_interface_->init()) {
    ROS_ERROR("Failed to initialized manipulation interface");
    return false;
  }

  observer_ = std::make_unique<StateObserver>(node_handle, man_interface_->fixed_base_);
  started_ = false;
  return true;
}

bool RoyalPandaControllerRos::init_parameters(ros::NodeHandle& node_handle) {
  if (!node_handle.getParam("arm_id", arm_id_)) {
    ROS_ERROR("RoyalPandaControllerRos: Could not read parameter arm_id");
    return false;
  }

  if (!node_handle.getParam("joint_names", joint_names_) || joint_names_.size() != 7) {
    ROS_ERROR(
        "RoyalPandaControllerRos: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR(
        "RoyalPandaControllerRos:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
    ROS_ERROR(
        "RoyalPandaControllerRos:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("RoyalPandaControllerRos: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("RoyalPandaControllerRos: coriolis_factor not found. Defaulting to "
                    << coriolis_factor_);
  }

  node_handle.param<bool>("debug", debug_, true);
  if (debug_) ROS_WARN_STREAM("In debug mode: running controller without sending commands.");

  return true;
}

bool RoyalPandaControllerRos::init_interfaces(hardware_interface::RobotHW* robot_hw) {
  // Model interface: returns kino-dynamic properties of the manipulator
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("RoyalPandaControllerRos: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id_ + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "RoyalPandaControllerRos: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  // Get state interface.
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("RoyalPandaControllerRos: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id_ + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "RoyalPandaControllerRos: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  // Effort interface for compliant control
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("RoyalPandaControllerRos: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names_[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("RoyalPandaControllerRos: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  return true;
}

bool RoyalPandaControllerRos::init_publishers(ros::NodeHandle& node_handle) {
  torques_publisher_.init(node_handle, "torque_comparison", 1);
  base_twist_publisher_.init(node_handle, "/cmd_vel", 1);
  return true;
}

void RoyalPandaControllerRos::starting(const ros::Time& time) {
  if (started_) return;

  arm_state_ = state_handle_->getRobotState();

  observer_->set_arm_state(arm_state_);
  observer_->get_state(x_);

  man_interface_->set_observation(x_, time.toSec());
  man_interface_->start();
  started_ = true;
}

void RoyalPandaControllerRos::update(const ros::Time& time, const ros::Duration& period) {
  observer_->set_arm_state(arm_state_);
  if (!observer_->get_state(x_)){
    ROS_WARN_THROTTLE(2.0, "[RoyalPandaControllerRos::update]: failed to assemble state.");
    return;
  }

  man_interface_->set_observation(x_, time.toSec());
  man_interface_->get_input(x_, u_, time.toSec());

  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> gravity = model_handle_->getGravity();

  double alpha = 0.99;
  for (size_t i = 0; i < 7; i++) {
    dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state.dq[i];
  }

  std::array<double, 7> tau_d_calculated;
  for (size_t i = 0; i < 7; ++i) {
    tau_d_calculated[i] = coriolis_factor_ * coriolis[i];
    if (!debug_) {
      tau_d_calculated[i] += d_gains_[i] * (u_[i] - dq_filtered_[i]);
    }
    // tau_d_calculated[i] = coriolis_factor_ * coriolis[i] +
                          // k_gains_[i] * (robot_state.q_d[i] - robot_state.q[i]) +
                          // d_gains_[i] * (robot_state.dq_d[i] - dq_filtered_[i]);
  }

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);
  }

  if (rate_trigger_() && torques_publisher_.trylock()) {
    std::array<double, 7> tau_j = robot_state.tau_J;
    std::array<double, 7> tau_error;
    double error_rms(0.0);
    for (size_t i = 0; i < 7; ++i) {
      tau_error[i] = last_tau_d_[i] - tau_j[i];
      error_rms += std::sqrt(std::pow(tau_error[i], 2.0)) / 7.0;
    }
    torques_publisher_.msg_.root_mean_square_error = error_rms;
    for (size_t i = 0; i < 7; ++i) {
      torques_publisher_.msg_.tau_commanded[i] = last_tau_d_[i];
      torques_publisher_.msg_.tau_error[i] = tau_error[i];
      torques_publisher_.msg_.tau_measured[i] = tau_j[i];
    }
    torques_publisher_.unlockAndPublish();
  }

  for (size_t i = 0; i < 7; ++i) {
    last_tau_d_[i] = tau_d_saturated[i] + gravity[i];  // torque sent is already gravity compensated
  }
}

void RoyalPandaControllerRos::stopping(const ros::Time& time) {
  man_interface_->stop();
  started_ = false;
}

std::array<double, 7> RoyalPandaControllerRos::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}  // namespace mppi_manipulation

PLUGINLIB_EXPORT_CLASS(manipulation::RoyalPandaControllerRos,
                       controller_interface::ControllerBase)