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
    // this is mppi_ros/controller_interface.init(), which also calls mppi_manipulation/controller_interface.init_ros()
    // the interesting bit is that it loads the robot description and object description (see mppi_manipulation/load.launch file)
    ROS_ERROR("[ManipulationController::init] Failed to initialize the manipulation interface");
    return false;
  }
  ROS_INFO("[ManipulationController::init] Solver initialized correctly.");

  // franka start the controller twice. We need an additional 
  // variable to avoid a restart, after the controller has been started without being stopped
  started_ = false;
  started_twice_ = false;
  state_received_ = false;
  
  // we do not actively control the gripper
  x_.setZero(PandaDim::STATE_DIMENSION);
  x_nom_.setZero(PandaDim::STATE_DIMENSION);
  u_.setZero(PandaDim::INPUT_DIMENSION);
  velocity_measured_.setZero(10);
  velocity_filtered_.setZero(10);
  position_desired_.setZero(10);
  position_measured_.setZero(10);
  position_initial_.setZero(10);
  torque_desired_.setZero(7);

  ROS_INFO("[ManipulationController::init] Controller successfully initialized!");
  return true;
}

// not of interest
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

  position_error_max_.setZero(10);
  std::vector<double> position_error_max;
  if (!node_handle.getParam("max_position_error", position_error_max) ||
      position_error_max.size() != 10) {
    ROS_ERROR("Failed to get max_position_error parameter");
    return false;
  }
  for (int i = 0; i < 10; i++) position_error_max_[i] = position_error_max[i];

  if (!node_handle.getParam("simulation", simulation_)) {
    ROS_ERROR("simulation not found");
    return false;
  }

  if (!node_handle.getParam("fixed_base", fixed_base_)) {
    ROS_ERROR("fixed_base not found");
    return false;
  }

  if (!node_handle.getParam("debug", debug_)) {
    ROS_ERROR("debug not found");
    return false;
  }

  if (!node_handle.getParam("log_file_path", log_file_path_)) {
    ROS_ERROR("log_file_path not found");
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
  
  if(!node_handle.param<std::vector<double>>("base_cmd_threshold", base_cmd_threshold_, {}) || base_cmd_threshold_.size() !=3){
    ROS_ERROR("Failed to parse base_cmd_threshold or invalid");
    return false;
  }

  ROS_INFO("Parameters successfully initialized.");
  return true;
}

// not of interest
bool ManipulationController::init_interfaces(hardware_interface::RobotHW* robot_hw) {
  
  // get the effort interfaces from the robot
  // the ones we use to send torque commands to the arm
  auto* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR("Failed to get effort joint interface.");
    return false;
  }

  // get the joint handles interfaces from the robot 
  // the ones we use to read the current robot state
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(
          effort_joint_interface->getHandle(joint_names_[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("Failed to get joint handles: " << ex.what());
      return false;
    }
  }

  // the velocity joint interface
  // this will eventually be used only to control the simulated robot
  auto* velocity_joint_interface =
      robot_hw->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface == nullptr) {
    ROS_ERROR("Failed to get velocity joint interface.");
    return false;
  }

  // velocity interfaces are supported only for the simulated robot hardware
  // We auto-detect that we are not in sim, when we cannot retrieve the 
  // base joint velocity interface handles
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

  // this interface provides access to the handle used to query 
  // dynamic information from the robot (e.g gravity and coriolis)
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR("Failed to get model interface.");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id_ + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("Failed to get model handle from interface: " << ex.what());
    return false;
  }

  // this interface is the custom franka counterpart of the jointStateInterface
  // but has more stuff (e.g also Wrench measurements from the end effector)
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("Failed to get state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id_ + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("Failed to get state handle from interface: " << ex.what());
    return false;
  }

  ROS_INFO("Interfaces successfully initialized.");
  return true;
}

// not of interest
void ManipulationController::init_ros(ros::NodeHandle& nh) {
  // initialize all ros publishers
  base_twist_publisher_.init(nh, base_twist_topic_, 1);
  nominal_state_publisher_.init(nh, nominal_state_topic_, 1);
  
  input_publisher_.init(nh, "vel_desired", 1);
  input_publisher_.msg_.data.resize(10, 0.0);
  
  position_desired_publisher_.init(nh, "pos_desired", 1);
  position_desired_publisher_.msg_.data.resize(10, 0.0);

  state_subscriber_ = nh.subscribe(state_topic_, 1, &ManipulationController::state_callback, this);

  ROS_INFO_STREAM("Sending base commands to: " << base_twist_topic_);
  ROS_INFO_STREAM("Receiving state at: " << state_topic_);
  ROS_INFO_STREAM("Ros successfully initialized.");
}

void ManipulationController::state_callback(
    const manipulation_msgs::StateConstPtr& state_msg) {
  if (!started_) return;

  {
    std::unique_lock<std::mutex> lock(observation_mutex_);
    manipulation::conversions::msgToEigen(*state_msg, x_, measurement_time_);
    
    if (!state_received_) {
      ROS_INFO("First call to state callback");
      start_time_ = measurement_time_;
      position_desired_[0] = state_msg->base_pose.x;
      position_desired_[1] = state_msg->base_pose.y;
      position_desired_[2] = state_msg->base_pose.z;
      position_initial_.head<3>() = position_desired_.head<3>();
      if (!man_interface_->init_reference_to_current_pose(x_, 0.0)) {
        ROS_WARN("Failed to set the controller reference to current state.");
    }
  }

    // zero base controller time
    observation_time_ = measurement_time_ - start_time_;
    man_interface_->set_observation(x_, observation_time_);
    man_interface_->update_reference(x_, observation_time_);

    // get the rotation corresponding to the latest observation
    getRotationMatrix(R_world_base_, x_(2));
  }

  x_ros_ = *state_msg;
  state_received_ = true;
}

void ManipulationController::starting(const ros::Time& time) {
  if (record_bag_) {
    bag_.open(bag_path_, rosbag::bagmode::Write);
  }

  if (started_) {
    ROS_INFO("Controller already started!");
    started_twice_ = true;
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

  for (size_t i = 0; i < 7; i++) {
    position_desired_[i + 3] = joint_handles_[i].getPosition();
    position_initial_[i + 3] = joint_handles_[i].getPosition();
  }
  position_measured_ = position_desired_;

  // metrics
  stage_cost_ = 0.0;

  // logging
  if (logging_) {
    ROS_INFO("Initializing logging!");
    signal_logger::setSignalLoggerStd();
    signal_logger::SignalLoggerOptions silo_options;
    signal_logger::logger->initLogger(silo_options);
    signal_logger::logger->stopLogger();

    // add all variables to be logged
    signal_logger::add(x_, "state");
    signal_logger::add(x_nom_, "nominal_state");
    signal_logger::add(u_, "velocity_mppi");
    signal_logger::add(torque_desired_, "torque_command");
    signal_logger::add(velocity_measured_, "velocity_measured");
    signal_logger::add(velocity_filtered_, "velocity_filtered");
    signal_logger::add(position_measured_, "position_measured");
    signal_logger::add(position_desired_, "position_desired");
    signal_logger::add(stage_cost_, "stage_cost");
    signal_logger::logger->startLogger(true);
  }
  started_ = true;
  ROS_INFO("Controller started!");
}


void ManipulationController::update_position_reference(
    const ros::Duration& period) {
  //clang-format off
  // input = [ base_velocity (3), arm velocity (7), gripper velocity (1)]
  // -> we discard gripper velocity as it is not supported (not used in the
  // physics engine)
  position_desired_.tail<7>() += u_.segment<7>(3) * period.toSec();
  position_desired_.head<3>() += R_world_base_ * u_.head<3>() * period.toSec();
  position_desired_ = position_measured_ + (position_desired_ - position_measured_)
                        .cwiseMax(-position_error_max_)
                        .cwiseMin(position_error_max_);
  //clang-format on
}

void ManipulationController::publish_ros(){
  if (input_publisher_.trylock()){
    for (int i=0; i<10; i++) input_publisher_.msg_.data[i] = u_[i];
      input_publisher_.unlockAndPublish();
  }

  if (position_desired_publisher_.trylock()) {
    for (int i = 0; i < 10; i++)
      position_desired_publisher_.msg_.data[i] = position_desired_[i];
    position_desired_publisher_.unlockAndPublish();
  }
}

void ManipulationController::send_command_base(const ros::Duration& period) {
  if (fixed_base_) return;

  // we compute a feedforward term based on the integral position error
  // not that we also have to convert to the base frame before sending the command
  Eigen::Vector3d u_ff = R_world_base_.transpose() * (position_desired_.head<3>() - position_measured_.head<3>());

  // In simulation, we can use directly the hardware interface
  if (has_base_handles_) {
    for (int i = 0; i < 3; i++) {
      base_joint_handles_[i].setCommand(gains_.base_gains.Ki[i] * u_ff[i] + u_[i]);
    }
    return;
  }

  if (base_trigger_() && base_twist_publisher_.trylock()) {
    Eigen::Vector3d twist_nominal(x_nom_ros_.base_twist.linear.x,
                                  x_nom_ros_.base_twist.linear.y,
                                  x_nom_ros_.base_twist.angular.z);

    double cmd_x = u_[0]  + gains_.base_gains.Ki[0] * u_ff[0];
    double cmd_y = u_[1]  + gains_.base_gains.Ki[1] * u_ff[1];
    double cmd_z = u_[2]  + gains_.base_gains.Ki[2] * u_ff[2];


    // we use a small threshold to avoid that small amplitude noise is sent to the base. 
    base_twist_publisher_.msg_.linear.x = (std::abs(cmd_x) > base_cmd_threshold_[0]) ? cmd_x - base_cmd_threshold_[0]: 0.0;
    base_twist_publisher_.msg_.linear.y = (std::abs(cmd_y) > base_cmd_threshold_[1]) ? cmd_y - base_cmd_threshold_[1]: 0.0;
    base_twist_publisher_.msg_.angular.z = (std::abs(cmd_z) > base_cmd_threshold_[2]) ? cmd_z - base_cmd_threshold_[2]: 0.0;
    base_twist_publisher_.unlockAndPublish();
  }
}

void ManipulationController::send_command_arm(const ros::Duration& period) {
  // clang-format off
  std::array<double, 7> tau_d_calculated{};
  if (simulation_){
    for (int i = 0; i < 7; ++i) {
      tau_d_calculated[i] = gains_.arm_gains.Ki[i] * (position_desired_[3+i] - robot_state_.q[i])
                          - gains_.arm_gains.Kd[i] * velocity_filtered_[3+i];  // in sim noisy velocity reference causes instability
    }  
  }
  else{
    // get coriolis from the model
    // we dont need the gravity term as well as the robot already compensates gravity by default
    std::array<double, 7> coriolis = model_handle_->getCoriolis();
    for (int i = 0; i < 7; ++i) {
      tau_d_calculated[i] = coriolis[i] + gains_.arm_gains.Ki[i] * (position_desired_[3+i] - robot_state_.q[i])
                                        + gains_.arm_gains.Kd[i] * (u_[3+i] - velocity_filtered_[3+i]);
    }
  }
  

  // max torque diff with sampling rate of 1 kHz is 1000 * (1 / sampling_time).
  // saturateTorqueRate(tau_d_calculated, robot_state_.tau_J_d, torque_desired_);
  // torque_desired_.setZero();
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_calculated[i]/*torque_desired_[i]*/);
  }
  // clang-format on
}

void ManipulationController::update(const ros::Time& time,
                                    const ros::Duration& period) {  
  if (!started_) {
    ROS_ERROR("[ManipulationController::update] Controller not started. Probably error occurred...");
    return;
  }

  if (!state_received_) {
    ROS_WARN_STREAM_THROTTLE(
        2.0, "[ManipulationController::update] State not received yet.");
    return;
  }

  current_time_ = time.toSec() - start_time_;
  ROS_DEBUG_STREAM("Ctl state:"
                   << std::endl
                   << std::setprecision(2)
                   << manipulation::conversions::eigenToString(x_));

  if (sequential_) {
    man_interface_->update_policy();
    man_interface_->get_input_state(x_, x_nom_, u_, current_time_);
    man_interface_->publish_ros_default();
    man_interface_->publish_ros();
  } else {
    man_interface_->get_input_state(x_, x_nom_, u_, current_time_);
  }

  manipulation::conversions::eigenToMsg(x_nom_, current_time_, x_nom_ros_);
  robot_state_ = state_handle_->getRobotState();
  
  if (record_bag_){  // WARN - this might quite slow down the controller
    optimal_rollout_.clear();
    man_interface_->get_controller()->get_optimal_rollout(optimal_rollout_);
    mppi_ros::to_msg(optimal_rollout_, optimal_rollout_ros_, true, 30);
    u_curr_ros_.data.assign(u_.data(), u_.data() + u_.size()); 
    bag_.write("optimal_rollout", time, optimal_rollout_ros_);
    bag_.write("current_input", time, u_curr_ros_);
  }

  // filter velocity measurements
  static const double alpha = 0.99;
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
  update_position_reference(period);
//  send_command_arm(period);
//  send_command_base(period);
  publish_ros();
  
  {
    std::unique_lock<std::mutex> lock(observation_mutex_);
    stage_cost_ = man_interface_->get_stage_cost(x_, u_, current_time_);
  }

  if (logging_ && log_counter_ == log_every_steps_){
    signal_logger::logger->collectLoggerData();
    log_counter_ = 0;
  }
  log_counter_++;
}

void ManipulationController::stopping(const ros::Time& time) {
  ROS_INFO("[ManipulationController::stopping] Stopping controller.");
  if(record_bag_) {
    ROS_INFO("[ManipulationController::stopping] Closing bag...");
    bag_.close();
  }

  if (logging_ && started_twice_){
    ROS_INFO_STREAM("Saving log data to: " << log_file_path_);
    signal_logger::logger->stopLogger();
    signal_logger::logger->saveLoggerData({signal_logger::LogFileType::BINARY},
                                          log_file_path_);
    signal_logger::logger->cleanup();
    ROS_INFO("Done!");
  }
}

void ManipulationController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d,
    Eigen::VectorXd& torque_cmd) {  // NOLINT (readability-identifier-naming)
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
