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

  if (!debug_){
    man_interface_ = std::make_unique<manipulation::PandaControllerInterface>(node_handle);
    if (!man_interface_->init()) {
      ROS_ERROR("Failed to initialized manipulation interface");
      return false;
    }
    ROS_INFO("Controller successfully initialized!");
  }

  started_ = false;
  return true;
}

bool RoyalPandaControllerRos::init_parameters(ros::NodeHandle& node_handle) {
  if (!node_handle.getParam("fixed_base", fixed_base_)) {
    ROS_ERROR("RoyalPandaControllerRos: Could not read parameter fixed_base");
    return false;
  }

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

  std::vector<double> base_gains;
  if (!node_handle.getParam("base_gains", base_gains) || base_gains.size() != 3) {
    ROS_ERROR(
        "RoyalPandaControllerRos:  Invalid or no base_gains parameters provided, aborting "
        "controller init!");
    return false;
  }
  base_gains_ << base_gains[0], base_gains[1], base_gains[2];

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
  if (debug_) ROS_WARN_STREAM("In debug mode: running controller with debug velocity commands.");

  node_handle.param<double>("debug_amplitude", amplitude_, 0.0);
  if (debug_) ROS_WARN_STREAM("Debug amplitude is: " << amplitude_);

  ROS_INFO("[RoyalPandaControllerRos::init_parameters]: Parameters successfully initialized.");
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

  ROS_INFO("[RoyalPandaControllerRos::init_interfaces]: Interfaces successfully initialized.");
  return true;
}

bool RoyalPandaControllerRos::init_ros(ros::NodeHandle& node_handle) {
  torques_publisher_.init(node_handle, "torque_comparison", 1);
  base_twist_publisher_.init(node_handle, base_twist_topic_, 1);
  state_subscriber_ =
      node_handle.subscribe(state_topic_, 1, &RoyalPandaControllerRos::state_callback, this);
  ROS_INFO_STREAM("Sending base commands to: " << base_twist_topic_ << std::endl
                  << "Received state at: " << state_topic_);
  nominal_state_publisher_.init(node_handle, "/x_nom", 1);
  return true;
}

void RoyalPandaControllerRos::state_callback(const manipulation_msgs::StateConstPtr& state_msg) {
  if (!started_) return;

  manipulation::conversions::msgToEigen(*state_msg, x_);
  man_interface_->set_observation(x_, ros::Time::now().toSec());
  x_ros_ = *state_msg;
  
  if (!state_received_) {
    state_received_ = true;
    state_last_receipt_time_ = ros::Time::now().toSec();
  } else {
    double current_time = ros::Time::now().toSec();
    double elapsed = current_time - state_last_receipt_time_;
    if (elapsed > 0.01) {
      state_ok_ = false;
      ROS_WARN_STREAM_THROTTLE(2.0, 
          "[RoyalPandaControllerRos::state_callback] State update is too slow. Current delay: "
          << elapsed);
    }
    state_last_receipt_time_ = current_time;
  }
}

void RoyalPandaControllerRos::starting(const ros::Time& time) {
  if (started_) return;
  
  if (!debug_ && !started_){
    if (!man_interface_->start()) {
      ROS_ERROR("[RoyalPandaControllerRos::starting] Failed  to start controller");
      started_ = false;
      return;
    }
    started_ = true;
  }

  // initial configuration
  robot_state_ = state_handle_->getRobotState();
  for (size_t i=0; i<7; i++){
    qd_[i] = robot_state_.q[i];
  }

  start_time_ = time.toSec();
  started_ = true;
}

void RoyalPandaControllerRos::update(const ros::Time& time, const ros::Duration& period) {
  if (!started_) {
    ROS_ERROR_ONCE("[RoyalPandaControllerRos::update]: Controller not started. Probably some error occourred...");
    return;
  }

  if (!state_received_) {
    ROS_WARN_STREAM_THROTTLE(2.0, "[RoyalPandaControllerRos::update]State not received yet.");
    return;
  }

  if (!debug_){
    //man_interface_->get_input(x_, u_, time.toSec());
    man_interface_->get_input_state(x_, x_nom_, u_, time.toSec());
    conversions::eigenToMsg(x_nom_, x_nom_ros_);
  }

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
      qd_[i] += u_.tail<8>()(i) * period.toSec();
      tau_d_calculated[i] = coriolis_factor_ * coriolis[i] +
                            k_gains_[i] * (qd_[i] - robot_state_.q[i]) +
                            d_gains_[i] * (u_.tail<8>()(i) - dq_filtered_[i]);
    } 
    else {
      double dt = ros::Time::now().toSec() - start_time_;
      double speed_desired = 0;
      double position_desired = qd_[i];
      if (i==0){
        position_desired += amplitude_ * std::sin(2 * M_PI * dt * frequency_);
        speed_desired = amplitude_ * 2 * M_PI * frequency_ * std::cos(2 * M_PI * dt * frequency_);
      }
      tau_d_calculated[i] = coriolis_factor_ * coriolis[i] +
                            k_gains_[i] * (position_desired - robot_state_.q[i]) +
                            d_gains_[i] * (speed_desired - dq_filtered_[i]);
    }
  }
  
  // send velocity commands to the base
  if (!fixed_base_ && !debug_) {
    if (base_trigger_() && base_twist_publisher_.trylock()) {
//      double alpha_base = 0.99;
//      double vx = alpha_base * state_ros_.base_twist.linear.x +  (1-alpha_base) * u_[0];
//      double vy = alpha_base * state_ros_.base_twist.linear.y +  (1-alpha_base) * u_[1];
//      double omega = alpha_base * state_ros_.base_twist.angular.z +  (1-alpha_base) * u_[2];

      // error is in the world frame
      // Eigen::Vector3d base_error(x_nom_ros_.base_pose.x - x_ros_.base_pose.x,
      //                            x_nom_ros_.base_pose.y - x_ros_.base_pose.y,
      //                            x_nom_ros_.base_pose.z - x_ros_.base_pose.z);
      // Eigen::Matrix3d r_world_base(Eigen::AngleAxisd(x_ros_.base_pose.z, Eigen::Vector3d::UnitZ()));
      // ROS_INFO_STREAM_THROTTLE(1.0, "Current base error: " << base_error);

      // Eigen::Vector3d vel_cmd = base_gains_.cwiseProduct(r_world_base.transpose() * base_error);

      // base_twist_publisher_.msg_.linear.x = vel_cmd.x();
      // base_twist_publisher_.msg_.linear.y = vel_cmd.y();
      // base_twist_publisher_.msg_.angular.z = vel_cmd.z();

      Eigen::Vector3d twist_nominal(x_nom_ros_.base_twist.linear.x,
                                    x_nom_ros_.base_twist.linear.y,
                                    x_nom_ros_.base_twist.angular.z);
      Eigen::Matrix3d r_world_base(Eigen::AngleAxisd(x_ros_.base_pose.z, Eigen::Vector3d::UnitZ()));
      Eigen::Vector3d twist_cmd = r_world_base.transpose() * twist_nominal;

      base_twist_publisher_.msg_.linear.x = twist_cmd.x();
      base_twist_publisher_.msg_.linear.y = twist_cmd.y();
      base_twist_publisher_.msg_.angular.z = twist_cmd.z();
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

  if (rate_trigger_()) {  
    if (torques_publisher_.trylock()){
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

    if (nominal_state_publisher_.trylock()){
      nominal_state_publisher_.msg_ = x_nom_ros_;
      nominal_state_publisher_.unlockAndPublish();
    }
  }

  for (size_t i = 0; i < 7; ++i) {
    last_tau_d_[i] = tau_d_saturated[i] + gravity[i];  // torque sent is already gravity compensated
  }
}

void RoyalPandaControllerRos::stopping(const ros::Time& time) {
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