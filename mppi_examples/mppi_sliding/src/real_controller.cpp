//
// Created by giuseppe on 22.01.21.
//

#include <mppi_sliding/real_controller.h>
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
using namespace manipulation_panda;

bool ManipulationController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& controller_nh) {

  if (!init_parameters(controller_nh)) return false;
  if (!init_interfaces(robot_hw)) return false;
  init_ros(controller_nh);

  // init interface
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

  std::string pkg_name = "mppi_sliding";

  if (!node_handle.getParam("arm_id", arm_id_)) {
    ROS_ERROR("Could not read parameter arm_id");
    return false;
  }

  // if (!node_handle.getParam("log_every_steps", log_every_steps_)) {
  //   ROS_ERROR("log_every_steps not found");
  //   return false;
  // }

  if (!node_handle.getParam("joint_names", joint_names_) ||
      joint_names_.size() != 7) {
    ROS_ERROR("Invalid or no joint_names parameters provided");
    return false;
  }


  if (!node_handle.getParam( pkg_name + "/sequential", sequential_)) {
    ROS_ERROR("Failed to get sequential parameter");
    return false;
  }

  max_position_error_.setZero(7);
  std::vector<double> max_position_error;
  if (!node_handle.getParam( pkg_name + "/max_position_error", max_position_error) ||
      max_position_error.size() != 7) {
    ROS_ERROR("Failed to get max_position_error parameter");
    return false;
  }
  for (int i = 0; i < 10; i++) max_position_error_[i] = max_position_error[i];

  if (!gains_.init_from_ros(node_handle, "controller_")) {
    ROS_ERROR("Failed to parse gains.");
    return false;
  }

  if (!node_handle.getParam( pkg_name + "state_topic", state_topic_)) {
    ROS_ERROR("state_topic not found");
    return false;
  }

  if (!node_handle.getParam( pkg_name + "nominal_state_topic", nominal_state_topic_)) {
    ROS_ERROR("nominal_state_topic not found");
    return false;
  }

  if (!node_handle.getParam( pkg_name + "record_bag", record_bag_)) {
    ROS_ERROR("record_bag not found");
    return false;
  }

  if (!node_handle.getParam( pkg_name + "bag_path", bag_path_)) {
    ROS_ERROR("bag_path not found");
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
        state_interface->getHandle(arm_id_ + "_robot"));   // this is the default state interface handle name, for details check  https://frankaemika.github.io/docs/franka_ros.html#writing-your-own-controller
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR("Failed to get state handle from interface: ", ex.what());
    return false;
  }

  ROS_INFO("Interfaces successfully initialized.");
  return true;
}

void ManipulationController::init_ros(ros::NodeHandle& nh) {

  nominal_state_publisher_.init(nh, nominal_state_topic_, 1);

  state_subscriber_ = nh.subscribe(
      state_topic_, 1, &ManipulationController::state_callback, this);

  ROS_INFO_STREAM("Receiving state at: " << state_topic_);
  ROS_INFO_STREAM("Ros successfully initialized.");
}

// TODO: use this interface as the percpetion input to object state in X_
void ManipulationController::state_callback(
    const manipulation_msgs::StateConstPtr& state_msg) {
  if (!started_) return;

  {
    std::unique_lock<std::mutex> lock(observation_mutex_);
    // modify the msgToEigen to adapt to panda
    manipulation::conversions::msgToEigen_panda(*state_msg, x_, observation_time_);
    
    if (!state_received_) {
      // position_desired_[0] = state_msg->base_pose.x;
      // position_desired_[1] = state_msg->base_pose.y;
      // position_desired_[2] = state_msg->base_pose.z;
      // position_initial_.head<3>() = position_desired_.head<3>();
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
  if (record_bag_) {
    bag_.open(bag_path_, rosbag::bagmode::Write);
  }

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
  x_.setZero(PandaDim::STATE_DIMENSION);
  x_nom_.setZero(PandaDim::STATE_DIMENSION);
  u_.setZero(PandaDim::INPUT_DIMENSION);
  u_opt_.setZero(PandaDim::INPUT_DIMENSION);

  velocity_measured_.setZero(PandaDim::INPUT_DIMENSION);
  velocity_filtered_.setZero(PandaDim::INPUT_DIMENSION);
  position_desired_.setZero(PandaDim::INPUT_DIMENSION);
  position_measured_.setZero(PandaDim::INPUT_DIMENSION);
  position_initial_.setZero(PandaDim::INPUT_DIMENSION);

  for (size_t i = 0; i < 7; i++) {
    position_desired_[i] = joint_handles_[i].getPosition();
    position_initial_[i] = joint_handles_[i].getPosition();

    // init man_interface state
    x_[i] = joint_handles_[i].getPosition();
  }
  
  position_measured_ = position_desired_;

  // metrics
  stage_cost_ = 0.0;

  // logging
  {
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
    // signal_logger::add(power_channels_, "power_channels");
    // signal_logger::add(power_from_error_, "power_from_error");
    // signal_logger::add(power_from_interaction_, "power_from_interaction");
    // signal_logger::add(total_power_exchange_, "total_power_exchange");
    signal_logger::add(external_torque_, "external_torque");
    signal_logger::logger->startLogger(true);
  }
  started_ = true;
  ROS_INFO("Controller started!");
}

void ManipulationController::update_position_reference(
    const ros::Duration& period) {
  position_desired_.head<7>() += u_opt_.tail<7>() * period.toSec();
  position_desired_ =
      position_measured_ + (position_desired_ - position_measured_)
                               .cwiseMax(-max_position_error_)
                               .cwiseMin(max_position_error_);
}


void ManipulationController::send_command_arm(const ros::Duration& period) {
  // clang-format off
  std::array<double, 7> tau_d_calculated{};
  for (int i = 0; i < 7; ++i) {
    tau_d_calculated[i] = gains_.arm_gains.Ki[i] * (position_desired_[i] - robot_state_.q[i])
        - gains_.arm_gains.Kd[i] * velocity_filtered_[i];
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
    man_interface_->update_reference(x_, time.toSec());
  }

  ROS_DEBUG_STREAM("Ctl state:"
                   << std::endl
                   << std::setprecision(2)
                   << manipulation::conversions::eigenToString_panda(x_));

  if (sequential_) {
    man_interface_->update_policy();
    man_interface_->get_input_state(x_, x_nom_, u_, time.toSec());
    man_interface_->publish_ros_default();
    man_interface_->publish_ros();
  } else {
    man_interface_->get_input_state(x_, x_nom_, u_, time.toSec());
  }

  // samilar as before, modify the eigenToMsg to adapt to panda
  manipulation::conversions::eigenToMsg_panda(x_nom_, time.toSec(), x_nom_ros_);
  // get state from pandaHW
  robot_state_ = state_handle_->getRobotState();

  if (record_bag_){
    optimal_rollout_.clear();
    man_interface_->get_controller()->get_optimal_rollout(optimal_rollout_);
    mppi_ros::to_msg(optimal_rollout_, optimal_rollout_ros_, true, 30);
    u_curr_ros_.data.assign(u_.data(), u_.data() + u_.size()); 
    bag_.write("optimal_rollout", time, optimal_rollout_ros_);
    bag_.write("current_input", time, u_curr_ros_);
  }

  static const double alpha = 0.1;

  // update state x_
  for (int i = 0; i < 7; i++) {

    x_[i] = robot_state_.q[i];
    x_[i+BASE_ARM_GRIPPER_DIM] = robot_state_.dq[i];

    position_measured_[i] = robot_state_.q[i];
    velocity_measured_[i] = robot_state_.dq[i];
    velocity_filtered_[i] =
        (1 - alpha) * velocity_filtered_[i] + alpha * robot_state_.dq[i];
  }

  //enforce_constraints(period);
  {
    u_opt_ = u_.head<7>();
  }
  update_position_reference(period);
  send_command_arm(period);
  //send_command_base(period);

  // what is this for?
  if (nominal_state_publisher_.trylock()) {
    nominal_state_publisher_.msg_ = x_nom_ros_;
    nominal_state_publisher_.unlockAndPublish();
  }

  {
    std::unique_lock<std::mutex> lock(observation_mutex_);
    stage_cost_ = man_interface_->get_stage_cost(x_, u_opt_, time.toSec());
  }

  // TODO(Boyang): this I understand as a debug print out, for me I don't have it
  //  but could write one
  // Eigen::Matrix<double, 6, 1> tracking_error =
  //     man_interface_->get_tracking_error();
  // std::cout << "lin err: " << tracking_error.head<3>().transpose() *
  // tracking_error.head<3>() << std::endl; std::cout << "ang err: " <<
  // tracking_error.tail<3>().transpose() * tracking_error.tail<3>() <<
  // std::endl;

  if (log_counter_ == log_every_steps_){
    signal_logger::logger->collectLoggerData();
    log_counter_ = 0;
  }
  log_counter_++;
}

void ManipulationController::stopping(const ros::Time& time) {
  if(record_bag_) {
    ROS_INFO("Closing bag...");
    bag_.close();
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
  R << std::cos(theta), -std::sin(theta), 0.0,
       std::sin(theta), std::cos(theta), 0.0,
       0.0, 0.0, 1.0;
  // clang-format on
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
  // total_power_exchange_ = power_from_error_ + power_from_interaction_;
  // energy_tank_.step(total_power_exchange_, period.toSec());

  // {
  //   std::unique_lock<std::mutex> lock(observation_mutex_);
  //   safety_filter_->update(x_, u_, ros::Time::now().toSec());

  //   // this is required for computing some metrics
  //   // we provide only the joints position (no gripper) since the implementation
  //   // of the joint limits in the safety_filter package assumes that the state
  //   // vector is eventually only the joint state
  //   // TODO(giuseppe) each problem shuould have its implementation (maybe
  //   // inherithed)
  //   //   as this is not working anymore if another constraint requires the full
  //   //   state instead.
  //   safety_filter_->update_violation(x_.head<10>());
  // }

  // compute new optimal input
  auto start = std::chrono::steady_clock::now();
  //bool filter_ok = safety_filter_->apply(u_filter_);
  auto end = std::chrono::steady_clock::now();
  opt_time_ = std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count() / 1.0e9;

}


PLUGINLIB_EXPORT_CLASS(manipulation_panda::ManipulationController,
                       controller_interface::ControllerBase)