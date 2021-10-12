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
#include <chrono>

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

  // we do not actively control the gripper
  x_.setZero(PandaDim::STATE_DIMENSION);
  xfirst_.setZero(PandaDim::STATE_DIMENSION);
  x_nom_.setZero(PandaDim::STATE_DIMENSION);
  u_.setZero(PandaDim::INPUT_DIMENSION);
  u_opt_.setZero(10);
  u_filter_.setZero(10);
  velocity_measured_.setZero(10);
  velocity_filtered_.setZero(10);
  position_desired_.setZero(10);
  position_measured_.setZero(10);
  position_initial_.setZero(10);

  ROS_INFO("Controller successfully initialized!");
  return true;
}

bool ManipulationController::init_parameters(ros::NodeHandle& node_handle) {
  if (!node_handle.getParam("arm_id", arm_id_)) {
    ROS_ERROR("Could not read parameter arm_id");
    return false;
  }

  if (!node_handle.getParam("log_every_steps", log_every_steps_)) {
    ROS_ERROR("log_every_steps not found");
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

  max_position_error_.setZero(10);
  std::vector<double> max_position_error;
  if (!node_handle.getParam("max_position_error", max_position_error) ||
      max_position_error.size() != 10) {
    ROS_ERROR("Failed to get max_position_error parameter");
    return false;
  }
  for (int i = 0; i < 10; i++) max_position_error_[i] = max_position_error[i];

  if (!node_handle.getParam("fixed_base", fixed_base_)) {
    ROS_ERROR("fixed_base not found");
    return false;
  }

  if (!node_handle.getParam("logging", logging_)) {
    ROS_ERROR("logging not found");
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

  if (!node_handle.getParam("record_bag", record_bag_)) {
    ROS_ERROR("record_bag not found");
    return false;
  }

  if (!node_handle.getParam("bag_path", bag_path_)) {
    ROS_ERROR("bag_path not found");
    return false;
  }
  
  if (!node_handle.getParam("apply_filter", apply_filter_)) {
    ROS_ERROR("apply_filter not found");
    return false;
  }

  if (!node_handle.getParam("apply_filter_to_rollouts",
                            apply_filter_to_rollouts_)) {
    ROS_ERROR("apply_filter_to_rollouts not found");
    return false;
  }

  if (!safety_filter_params_.init_from_ros(node_handle)) {
    ROS_ERROR("Failed to parse safety filter parameters.");
    return false;
  }
  ROS_INFO_STREAM("Controller Safety Filter initialized\n"
                  << safety_filter_params_);

  ROS_INFO("Parameters successfully initialized.");
  return true;
}

bool ManipulationController::init_interfaces(
    hardware_interface::RobotHW* robot_hw) {
  // TO RESTORE
  // auto* effort_joint_interface =
  //     robot_hw->get<hardware_interface::EffortJointInterface>();
  // if (effort_joint_interface == nullptr) {
  //   ROS_ERROR("Failed to get effort joint interface.");
  //   return false;
  // }

  // for (size_t i = 0; i < 7; ++i) {
  //   try {
  //     joint_handles_.push_back(
  //         effort_joint_interface->getHandle(joint_names_[i]));
  //   } catch (const hardware_interface::HardwareInterfaceException& ex) {
  //     ROS_ERROR("Failed to get joint handles: ", ex.what());
  //     return false;
  //   }
  // }

  auto* velocity_joint_interface =
      robot_hw->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface == nullptr) {
    ROS_ERROR("Failed to get velocity joint interface.");
    return false;
  }

  has_base_handles_ = true;
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
    x_(STATE_DIMENSION - TORQUE_DIMENSION - 1) = energy_tank_.get_state();
    

    if (!state_received_) {
      ROS_INFO("First call to state callback");
      position_desired_[0] = state_msg->base_pose.x;
      position_desired_[1] = state_msg->base_pose.y;
      position_desired_[2] = state_msg->base_pose.z;
      position_initial_.head<3>() = position_desired_.head<3>();
      if (!man_interface_->init_reference_to_current_pose(x_,
                                                          observation_time_)) {
        ROS_WARN("Failed to set the controller reference to current state.");
      }
      xfirst_ = x_;
      ROS_INFO_STREAM("Resetting the state always at the first state: "
                      << xfirst_.transpose());
      start_time_ = state_msg->header.stamp.toSec();
    }
  }

  x_ros_ = *state_msg;
  state_received_ = true;
}

void ManipulationController::starting(const ros::Time& time) {
  if (record_bag_) {
    bag_.open(bag_path_, rosbag::bagmode::Write);
  }

  state_received_ = false;
  if (started_) {
    ROS_INFO("Controller already started!");
    return;
  }

  // with sequential execution the optimization needs to be explicitly called
  // in the update function of the controller (not real-time safe)
  if (!sequential_) {
    started_ = man_interface_->start();
    if (!started_) {
      ROS_ERROR("Failed  to start controller");
      return;
    }
  }

  // TO RESTORE
  // for (size_t i = 0; i < 7; i++) {
  //   position_desired_[i + 3] = joint_handles_[i].getPosition();
  //   position_initial_[i + 3] = joint_handles_[i].getPosition();
  // }
  // position_measured_ = position_desired_;

  // reset the energy tank initial state
  double initial_tank_state =
      std::sqrt(2.0 * safety_filter_params_.initial_tank_energy);
  energy_tank_.reset(initial_tank_state, time.toSec());

  // reset the safety filter
  safety_filter_ =
      std::make_unique<PandaMobileSafetyFilter>(safety_filter_params_);

  // this is the filter that is used to sanitize the rollouts
  if (apply_filter_to_rollouts_) {
    ROS_INFO("Applying safety filter to rollouts");
    mppi::filter_ptr safety_filter_ctrl =
        std::make_shared<PandaMobileSafetyFilter>(safety_filter_params_);
    man_interface_->get_controller()->set_filter(safety_filter_ctrl);
  }

  // metrics
  stage_cost_ = 0.0;

  // logging
  if (logging_) {
    signal_logger::logger->stopLogger();
    signal_logger::add(opt_time_, "opt_time");
    signal_logger::add(x_, "state");
    signal_logger::add(x_nom_, "nominal_state");
    signal_logger::add(u_, "velocity_mppi");
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
  }

  started_ = true;
  ROS_INFO("\n\n\n\nController started!\n\n\n");
}

void ManipulationController::enforce_constraints(const ros::Duration& period) {
  // compute the total power exchange with the tank
  external_torque_ = x_.tail<TORQUE_DIMENSION>().head<10>();
  power_channels_ = u_opt_.cwiseProduct(external_torque_);
  power_from_interaction_ = power_channels_.sum();

  // Only the arm has integral control
  power_from_error_ = 0.0;
  //(u_opt_ - velocity_filtered_).tail<7>().transpose() *
  //gains_.arm_gains.Ki.asDiagonal() *
  //                    (position_desired_ - position_initial_).tail<7>();
  total_power_exchange_ = power_from_error_ + power_from_interaction_;
  energy_tank_.step(total_power_exchange_, period.toSec());

  {
    std::unique_lock<std::mutex> lock(observation_mutex_);
    safety_filter_->update(x_, u_, ros::Time::now().toSec());

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

  // compute new optimal input
  auto start = std::chrono::steady_clock::now();
  bool filter_ok = safety_filter_->apply(u_filter_);
  auto end = std::chrono::steady_clock::now();
  opt_time_ = std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count() / 1.0e9;

  // safety filter does not account for the gripper -> extract only the base+arm
  // part
  if (apply_filter_ && filter_ok) {
    u_opt_ = u_filter_.head<10>();
  } else {
    u_opt_ = u_.head<10>();
  }
}

void ManipulationController::update_position_reference(
    const ros::Duration& period) {
  position_desired_.tail<7>() += u_opt_.tail<7>() * period.toSec();
  position_desired_.head<3>() +=
      R_world_base * u_opt_.head<3>() * period.toSec();
  position_desired_ =
      position_measured_ + (position_desired_ - position_measured_)
                               .cwiseMax(-max_position_error_)
                               .cwiseMin(max_position_error_);
}

void ManipulationController::send_command_base(const ros::Duration& period) {
  if (fixed_base_) return;

  // compute feedforward as function of the error in global position
  // and then convert to base frame
  Eigen::Vector3d u_ff = R_world_base.transpose() * (position_desired_.head<3>() - position_measured_.head<3>());

  // In simulation, we can use directly the hardware interface
  if (has_base_handles_) {
    for (int i = 0; i < 3; i++) {
      base_joint_handles_[i].setCommand(gains_.base_gains.Ki[i] * u_ff[i] + u_opt_[i]);
    }
    return;
  }

  if (base_trigger_() && base_twist_publisher_.trylock()) {
    Eigen::Vector3d twist_nominal(x_nom_ros_.base_twist.linear.x,
                                  x_nom_ros_.base_twist.linear.y,
                                  x_nom_ros_.base_twist.angular.z);

    base_twist_publisher_.msg_.linear.x = u_opt_[0] + gains_.base_gains.Ki[0] * u_ff[0];
    base_twist_publisher_.msg_.linear.y = u_opt_[1] + gains_.base_gains.Ki[1] * u_ff[1];
    base_twist_publisher_.msg_.angular.z = u_opt_[2] + gains_.base_gains.Ki[2] * u_ff[2];
    base_twist_publisher_.unlockAndPublish();
  }
}

void ManipulationController::send_command_arm(const ros::Duration& period) {
  // clang-format off
  std::array<double, 7> tau_d_calculated{};
  for (int i = 0; i < 7; ++i) {
    tau_d_calculated[i] = gains_.arm_gains.Ki[i] * (position_desired_[3+i] - robot_state_.q[i])
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

  {
    std::unique_lock<std::mutex> lock(observation_mutex_);
    man_interface_->set_observation(
        xfirst_,
        time.toSec());  // man_interface_->set_observation(x_, time.toSec());
    man_interface_->update_reference(
        xfirst_,
        time.toSec());  // man_interface_->update_reference(x_, time.toSec());
    getRotationMatrix(R_world_base,
                      xfirst_(2));  // getRotationMatrix(R_world_base, x_(2));
  }

  ROS_DEBUG_STREAM("Ctl state:"
                   << std::endl
                   << std::setprecision(2)
                   << manipulation::conversions::eigenToString(x_));

  if (sequential_) {
    man_interface_->update_policy();
    man_interface_->get_input_state(xfirst_, x_nom_, u_,
                                    time.toSec());  // man_interface_->get_input_state(x_,
                                                    // x_nom_, u_, time.toSec());
    man_interface_->publish_ros_default();
    man_interface_->publish_ros();
  } else {
    man_interface_->get_input_state(
        xfirst_, x_nom_, u_, time.toSec());  // man_interface_->get_input_state(x_,
                                             // x_nom_, u_, time.toSec());
  }

  manipulation::conversions::eigenToMsg(x_nom_, time.toSec(), x_nom_ros_);
  robot_state_ = state_handle_->getRobotState();
  
  if (record_bag_){
    optimal_rollout_.clear();
    man_interface_->get_controller()->get_optimal_rollout(optimal_rollout_);
    mppi_ros::to_msg(optimal_rollout_, optimal_rollout_ros_, true, 30);
    u_curr_ros_.data.assign(u_.data(), u_.data() + u_.size()); 
    bag_.write("optimal_rollout", time, optimal_rollout_ros_);
    bag_.write("current_input", time, u_curr_ros_);
  }

  // TO RESTORE
  // static const double alpha = 0.1;
  // position_measured_.head<3>() = x_.head<3>();
  // velocity_measured_.head<3>() = x_.segment<3>(12);
  // velocity_filtered_.head<3>() = (1 - alpha) * velocity_filtered_.head<3>() +
  //                                alpha * velocity_measured_.head<3>();
  // for (int i = 0; i < 7; i++) {
  //   position_measured_[i + 3] = robot_state_.q[i];
  //   velocity_measured_[i + 3] = robot_state_.dq[i];
  //   velocity_filtered_[i + 3] =
  //       (1 - alpha) * velocity_filtered_[i + 3] + alpha * robot_state_.dq[i];
  // }

  // enforce_constraints(period);
  // update_position_reference(period);
  // send_command_arm(period);
  // send_command_base(period);

  if (nominal_state_publisher_.trylock()) {
    nominal_state_publisher_.msg_ = x_nom_ros_;
    nominal_state_publisher_.unlockAndPublish();
  }

  {
    std::unique_lock<std::mutex> lock(observation_mutex_);
    stage_cost_ = man_interface_->get_stage_cost(x_, u_opt_, time.toSec());
  }

  if (log_counter_ == log_every_steps_ && logging_) {
    signal_logger::logger->collectLoggerData();
    log_counter_ = 0;
  }
  log_counter_++;
}

void ManipulationController::stopping(const ros::Time& time) {
  ROS_INFO("Stopping controller.");
  if(record_bag_) {
    ROS_INFO("Closing bag...");
    bag_.close();
  }
  //  signal_logger::logger->disableElement("/log/torque_command");
  //  signal_logger::logger->disableElement("/log/stage_cost");
  //  for (const auto& constraint : safety_filter_->constraints_) {
  //    signal_logger::logger->disableElement("/log/" + constraint.first +
  //                                          "_violation");
  //  }
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
  R << std::cos(theta), -std::sin(theta), 0.0,
       std::sin(theta), std::cos(theta), 0.0,
       0.0, 0.0, 1.0;
  // clang-format on
}

PLUGINLIB_EXPORT_CLASS(manipulation_royalpanda::ManipulationController,
                       controller_interface::ControllerBase)