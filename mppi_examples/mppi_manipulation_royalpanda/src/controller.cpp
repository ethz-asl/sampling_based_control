//
// Created by giuseppe on 22.01.21.
//

#include <mppi_manipulation_royalpanda/controller.h>
#include <mppi_manipulation_royalpanda/utils.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

#include <manipulation_msgs/conversions.h>

using namespace manipulation;
using namespace manipulation_royalpanda;

bool ManipulationController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  if (!init_parameters(node_handle)) return false;
  if (!init_interfaces(robot_hw)) return false;
  init_ros(node_handle);

  man_interface_ =
      std::make_unique<manipulation::PandaControllerInterface>(node_handle);
  if (!man_interface_->init()) {
    NAMED_LOG_ERROR("Failed to initialized manipulation interface");
    return false;
  }
  NAMED_LOG_INFO("Solver initialized correctly.");

  NAMED_LOG_INFO("Controller successfully initialized!");
  started_ = false;
  return true;
}

bool ManipulationController::init_parameters(ros::NodeHandle& node_handle) {
  if (!node_handle.getParam("arm_id", arm_id_)) {
    NAMED_LOG_ERROR("Could not read parameter arm_id");
    return false;
  }

  if (!node_handle.getParam("joint_names", joint_names_) ||
      joint_names_.size() != 7) {
    NAMED_LOG_ERROR("Invalid or no joint_names parameters provided");
    return false;
  }

  if (!node_handle.getParam("sequential", sequential_)) {
    NAMED_LOG_ERROR("Failed to get sequential parameter");
    return false;
  }

  if (!gains_.init_from_ros(node_handle)) {
    NAMED_LOG_ERROR("Failed to parse gains.");
    return false;
  }

  if (!node_handle.getParam("state_topic", state_topic_)) {
    NAMED_LOG_ERROR("state_topic not found");
    return false;
  }

  if (!node_handle.getParam("nominal_state_topic", nominal_state_topic_)) {
    NAMED_LOG_ERROR("nominal_state_topic not found");
    return false;
  }

  if (!node_handle.getParam("base_twist_topic", base_twist_topic_)) {
    NAMED_LOG_ERROR("base_twist_topic not found");
    return false;
  }

  NAMED_LOG_INFO("Parameters successfully initialized.");
  return true;
}

bool ManipulationController::init_interfaces(
    hardware_interface::RobotHW* robot_hw) {
  auto* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    NAMED_LOG_ERROR("Failed to get effort joint interface.");
    return false;
  }

  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(
          effort_joint_interface->getHandle(joint_names_[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      NAMED_LOG_ERROR("Failed to get joint handles: ", ex.what());
      return false;
    }
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    NAMED_LOG_ERROR("Failed to get model interface.");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id_ + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    NAMED_LOG_ERROR("Failed to get model handle from interface: ", ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    NAMED_LOG_ERROR("Failed to get state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id_ + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    NAMED_LOG_ERROR("Failed to get state handle from interface: ", ex.what());
    return false;
  }

  NAMED_LOG_INFO("Interfaces successfully initialized.");
  return true;
}

void ManipulationController::init_ros(ros::NodeHandle& nh) {
  base_twist_publisher_.init(nh, base_twist_topic_, 1);
  nominal_state_publisher_.init(nh, nominal_state_topic_, 1);

  state_subscriber_ = nh.subscribe(
      state_topic_, 1, &ManipulationController::state_callback, this);

  NAMED_LOG_INFO("Sending base commands to: ", base_twist_topic_);
  NAMED_LOG_INFO("Receiving state at: ", state_topic_);
  NAMED_LOG_INFO("Ros successfully initialized.");
}

void ManipulationController::state_callback(
    const manipulation_msgs::StateConstPtr& state_msg) {
  if (!started_) return;

  {
    std::unique_lock<std::mutex> lock(observation_mutex_);
    manipulation::conversions::msgToEigen(*state_msg, x_, observation_time_);
  }

  x_ros_ = *state_msg;

  if (!state_received_) {
    state_received_ = true;
  } else {
    if ((observation_time_ - last_observation_time_) > 0.01) {
      state_ok_ = false;
      NAMED_LOG_WARN("State update is too slow. Current delay: ",
                     (observation_time_ - last_observation_time_));
    }
  }
  last_observation_time_ = observation_time_;
}

void ManipulationController::starting(const ros::Time& time) {
  if (started_) return;

  if (!sequential_) {
    started_ = man_interface_->start();
    if (!started_) {
      NAMED_LOG_ERROR("Failed  to start controller");
      return;
    }
  }

  arm_velocity_filtered_.setZero(7);
  arm_position_desired_.setZero(7);
  for (size_t i = 0; i < 7; i++) {
    arm_position_desired_[i] = joint_handles_[i].getPosition();
  }
  started_ = true;
  NAMED_LOG_INFO("Controller started!");
}

void ManipulationController::send_command_base(const ros::Duration& period) {
  // in simulation we can use directly the hardware interface
  // TODO

  if (base_trigger_() && base_twist_publisher_.trylock()) {
    Eigen::Vector3d twist_nominal(x_nom_ros_.base_twist.linear.x,
                                  x_nom_ros_.base_twist.linear.y,
                                  x_nom_ros_.base_twist.angular.z);

    base_twist_publisher_.msg_.linear.x = u_[0];
    base_twist_publisher_.msg_.linear.y = u_[1];
    base_twist_publisher_.msg_.angular.z = u_[2];
    base_twist_publisher_.unlockAndPublish();
  }
}

void ManipulationController::send_command_arm(const ros::Duration& period) {
  robot_state_ = state_handle_->getRobotState();

  // clang-format off
  static double alpha = 0.99;
  for (int i = 0; i < 7; i++) {
    arm_velocity_filtered_[i] = (1 - alpha) * arm_velocity_filtered_[i] +
                                alpha * robot_state_.dq[i];
  }

  std::array<double, 7> tau_d_calculated{};
  arm_position_desired_ += u_.segment<7>(3) * period.toSec();
  for (int i = 0; i < 7; ++i) {
    tau_d_calculated[i] =
        gains_.arm_gains.Kp[i] * (arm_position_desired_[i] - robot_state_.q[i]) +
        gains_.arm_gains.Kd[i] * (u_.segment<7>(3)(i) - arm_velocity_filtered_[i]);
  }

  // max torque diff with sampling rate of 1 kHz is 1000 * (1 / sampling_time).
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated,
                                                             robot_state_.tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);
  }
  // clang-format on
}

void ManipulationController::update(const ros::Time& time,
                                    const ros::Duration& period) {
  if (!started_) {
    NAMED_LOG_ERROR("Controller not started. Probably error occurred...");
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

  if (sequential_) {
    man_interface_->update_policy();
    man_interface_->get_input(x_, u_, time.toSec());
    man_interface_->publish_ros_default();
    man_interface_->publish_ros();
  } else {
    man_interface_->get_input_state(x_, x_nom_, u_, time.toSec());
  }

  manipulation::conversions::eigenToMsg(x_nom_, time.toSec(), x_nom_ros_);

  send_command_arm(period);
  send_command_base(period);

  if (nominal_state_publisher_.trylock()) {
    nominal_state_publisher_.msg_ = x_nom_ros_;
    nominal_state_publisher_.unlockAndPublish();
  }
}

void ManipulationController::stopping(const ros::Time& time) {}

std::array<double, 7> ManipulationController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>&
        tau_J_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] +
        std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
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