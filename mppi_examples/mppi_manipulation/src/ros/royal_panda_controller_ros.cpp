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

#include <manipulation_msgs/conversions.h>

namespace manipulation {

bool RoyalPandaControllerRos::init(hardware_interface::RobotHW* robot_hw,
                                   ros::NodeHandle& node_handle) {
  if (!init_parameters(node_handle)) return false;
  if (!init_interfaces(robot_hw)) return false;
  if (!init_ros(node_handle)) return false;

  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);

  man_interface_ = std::make_unique<manipulation::PandaControllerInterface>(node_handle);
  if (!man_interface_->init()) {
    ROS_ERROR("Failed to initialized manipulation interface");
    return false;
  }

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

  if (!node_handle.getParam("state_topic", state_topic_)) {
    ROS_INFO_STREAM("RoyalPandaControllerRos: state_topic not found");
    return false;
  }

  if (!node_handle.getParam("base_twist_topic", base_twist_topic_)) {
    ROS_INFO_STREAM("RoyalPandaControllerRos: base_twist_topic not found");
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

bool RoyalPandaControllerRos::init_ros(ros::NodeHandle& node_handle) {
  torques_publisher_.init(node_handle, "torque_comparison", 1);
  base_twist_publisher_.init(node_handle, base_twist_topic_, 1);
  state_subscriber_ =
      node_handle.subscribe(state_topic_, 1, &RoyalPandaControllerRos::state_callback, this);
  ROS_INFO_STREAM("Sending base commands to: " << base_twist_topic_ << std::endl
                  << "Received state at: " << state_topic_);
  return true;
}

void RoyalPandaControllerRos::state_callback(const manipulation_msgs::StateConstPtr& state_msg) {
  manipulation::conversions::msgToEigen(*state_msg, x_);
  if (!state_received_) {
    state_ok_ = true;
    state_received_ = true;
    state_last_receipt_time_ = ros::Time::now().toSec();
  } else {
    double current_time = ros::Time::now().toSec();
    double elapsed = current_time - state_last_receipt_time_;
    if (elapsed > 0.01) {
      state_ok_ = false;
      ROS_ERROR_STREAM(
          "[RoyalPandaControllerRos::state_callback] State update is too slow. Current delay: "
          << elapsed);
    }
    state_last_receipt_time_ = ros::Time::now().toSec();
  }
}

void RoyalPandaControllerRos::starting(const ros::Time& time) {
  if (started_) return;
  if (!man_interface_->start()) {
    ROS_ERROR("[RoyalPandaControllerRos::starting] Failed  to initialize controller");
    started_ = false;
    return;
  }
  started_ = true;
}

void RoyalPandaControllerRos::update(const ros::Time& time, const ros::Duration& period) {
  if (!started_) {
    ROS_WARN("Controller not started.");
    return;
  }

  if (!state_received_) {
    ROS_WARN_STREAM_THROTTLE(2.0, "[RoyalPandaControllerRos::update]State not received yet.");
    return;
  }

  if (!state_ok_) {
    ROS_ERROR_ONCE("[RoyalPandaControllerRos::update] Failed to update the state. Stopping ....");
    stop_robot();
  }

  man_interface_->set_observation(x_, time.toSec());
  man_interface_->get_input(x_, u_, time.toSec());

  robot_state_ = state_handle_->getRobotState();
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> gravity = model_handle_->getGravity();

  double alpha = 0.99;
  for (size_t i = 0; i < 7; i++) {
    dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state_.dq[i];
  }

  std::array<double, 7> tau_d_calculated{};
  for (size_t i = 0; i < 7; ++i) {
    tau_d_calculated[i] = coriolis_factor_ * coriolis[i];
    if (!debug_) {
      tau_d_calculated[i] += d_gains_[i] * (u_.tail<8>()(i) - dq_filtered_[i]);
    } else {
      tau_d_calculated[i] = coriolis_factor_ * coriolis[i] +
                            // k_gains_[i] * (robot_state.q_d[i] - robot_state.q[i]) +
                            d_gains_[i] * (robot_state_.dq_d[i] - dq_filtered_[i]);
    }
  }

  // send velocity commands to the base
  if (!man_interface_->fixed_base_) {
    if (base_twist_publisher_.trylock()) {
      base_twist_publisher_.msg_.linear.x = u_[0];
      base_twist_publisher_.msg_.linear.y = u_[1];
      base_twist_publisher_.msg_.angular.z = u_[2];
      base_twist_publisher_.unlockAndPublish();
    }
  }

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  std::array<double, 7> tau_d_saturated =
      saturateTorqueRate(tau_d_calculated, robot_state_.tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);
  }

  if (rate_trigger_() && torques_publisher_.trylock()) {
    std::array<double, 7> tau_j = robot_state_.tau_J;
    std::array<double, 7> tau_error{};
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

void RoyalPandaControllerRos::stop_robot() {
  std::array<double, 7> tau_d_calculated{};
  tau_d_calculated.fill(0.0);
  robot_state_ = state_handle_->getRobotState();
  std::array<double, 7> tau_d_saturated =
      saturateTorqueRate(tau_d_calculated, robot_state_.tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);
  }
}
}  // namespace manipulation

PLUGINLIB_EXPORT_CLASS(manipulation::RoyalPandaControllerRos, controller_interface::ControllerBase)