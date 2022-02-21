//
// Created by boyang on 08.12.21.
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
  if (!node_handle.getParam("arm_id", arm_id_)) {
    ROS_ERROR("Could not read parameter arm_id");
    return false;
  }

  if (!node_handle.getParam("joint_names", joint_names_) ||
      joint_names_.size() != 7) {
    ROS_ERROR("Invalid or no joint_names parameters provided");
    return false;
  }

  if (!node_handle.getParam("sequential", sequential_)) {
    ROS_ERROR("Failed to get sequential parameter");
    return false;
  }

  max_position_error_.setZero(7);
  std::vector<double> max_position_error;
  if (!node_handle.getParam("max_position_error", max_position_error) ||
      max_position_error.size() != 7) {
    ROS_ERROR("Failed to get max_position_error parameter");
    return false;
  }
  for (int i = 0; i < 7; i++) max_position_error_[i] = max_position_error[i];

  if (!gains_.init_from_ros(node_handle, "controller_")) {
    ROS_ERROR("Failed to parse gains.");
    return false;
  }
  ROS_INFO_STREAM("gains is following: " << gains_);

  if (!node_handle.getParam("state_topic", state_topic_)) {
    ROS_ERROR("state_topic not found");
    return false;
  }
  ROS_INFO_STREAM("sub state topic from: " << state_topic_);

  if (!node_handle.getParam("table_state_topic", table_state_topic_)) {
    ROS_ERROR("table_state_topic not found");
    return false;
  }

  if (!node_handle.getParam("nominal_state_topic", nominal_state_topic_)) {
    ROS_ERROR("nominal_state_topic not found");
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
    state_handle_ = std::make_unique<
        franka_hw::FrankaStateHandle>(state_interface->getHandle(
        arm_id_ +
        "_robot"));  // this is the default state interface handle name, for
                     // details check
                     // https://frankaemika.github.io/docs/franka_ros.html#writing-your-own-controller
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR("Failed to get state handle from interface: ", ex.what());
    return false;
  }

  ROS_INFO("Interfaces successfully initialized.");
  return true;
}

void ManipulationController::init_ros(ros::NodeHandle& nh) {
  nominal_state_publisher_.init(nh, nominal_state_topic_, 1);
  stage_cost_publisher_.init(nh, "/stage_cost", 1);

  state_subscriber_ = nh.subscribe(
      state_topic_, 1, &ManipulationController::state_callback, this);

  ROS_INFO_STREAM("Receiving state at: " << state_topic_);
  ROS_INFO_STREAM("Ros successfully initialized.");
}

// TODO: use this interface as the percpetion input to object state in X_
void ManipulationController::state_callback(
    const manipulation_msgs::StateConstPtr& state_msg) {
  if (!started_) return;

  std::unique_lock<std::mutex> lock(observation_mutex_);
  // modify the msgToEigen to adapt to panda'
  manipulation::conversions::msgToEigen_panda(*state_msg, x_,
                                              observation_time_);
  observation_time_ = run_time.toSec();
  // ROS_INFO_STREAM(" position info after conversion: " << x_.transpose());

  if (!state_received_) {
    if (!man_interface_->init_reference_to_current_pose(x_,
                                                        observation_time_)) {
      ROS_WARN("Failed to set the controller reference to current state.");
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
  started_ = man_interface_->start();
  if (!started_) {
    ROS_ERROR("Failed  to start controller");
    return;
  }

  // we do not actively control the gripper
  x_.setZero(PandaDim::STATE_DIMENSION);
  x_nom_.setZero(PandaDim::STATE_DIMENSION);
  u_.setZero(PandaDim::INPUT_DIMENSION);
  u_opt_.setZero(PandaDim::INPUT_DIMENSION);
  x_nom_ros_.arm_state.position.resize(9);
  x_nom_ros_.arm_state.velocity.resize(9);
  x_nom_ros_.object_state.position.resize(7);
  x_nom_ros_.object_state.velocity.resize(7);

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
  stage_cost_.data = 0.0;

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
    signal_logger::add(stage_cost_.data, "stage_cost");
    signal_logger::logger->startLogger(true);
  }
  start_time = time;
  run_time.nsec = 0;
  run_time.sec = 0;
  ROS_INFO_STREAM("start at ros time: " << start_time);

  started_ = true;
  ROS_INFO("Controller started!");
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
    // ROS_INFO_STREAM("observed msg: " << x_.transpose());
    man_interface_->set_observation(x_, run_time.toSec());
    man_interface_->update_reference(x_, run_time.toSec());
  }

  // ROS_INFO_STREAM("Ctl state:"
  //                  << std::endl
  //                  << std::setprecision(2)
  //                  << manipulation::conversions::eigenToString_panda(x_));

  const auto p0 = std::chrono::time_point<std::chrono::high_resolution_clock>{};
  man_interface_->get_input_state(x_, x_nom_, u_, run_time.toSec());
  const auto p3 = std::chrono::high_resolution_clock::now();
  auto tstamp = p3 - p0;
  int32_t sec = std::chrono::duration_cast<std::chrono::microseconds>(tstamp).count();
  ROS_INFO_STREAM("duration of get input state: " << sec);



  // samilar as before, modify the eigenToMsg to adapt to panda
  manipulation::conversions::eigenToMsg_panda(x_nom_, run_time.toSec(),
                                              x_nom_ros_);
  // get state from pandaHW
  robot_state_ = state_handle_->getRobotState();

  if (record_bag_) {
    optimal_rollout_.clear();
    man_interface_->get_controller()->get_optimal_rollout(optimal_rollout_);
    mppi_ros::to_msg(optimal_rollout_, optimal_rollout_ros_, true, 30);
    u_curr_ros_.data.assign(u_.data(), u_.data() + u_.size());
    bag_.write("optimal_rollout", run_time, optimal_rollout_ros_);
    bag_.write("current_input", run_time, u_curr_ros_);
  }

  static const double alpha = 0.5;

  // update state x_
  for (int i = 0; i < 7; i++) {
    position_measured_[i] = robot_state_.q[i];
    velocity_measured_[i] = robot_state_.dq[i];
    velocity_filtered_[i] =
        (1 - alpha) * velocity_filtered_[i] + alpha * robot_state_.dq[i];
  }

  // retrieving only the arm commands
  u_opt_.head<7>() = u_.head<7>();

  update_position_reference(period);
  send_command_arm(period);

  if (nominal_state_publisher_.trylock()) {
    nominal_state_publisher_.msg_ = x_nom_ros_;
    nominal_state_publisher_.unlockAndPublish();
  }

  {
    std::unique_lock<std::mutex> lock(observation_mutex_);
    stage_cost_.data = man_interface_->get_stage_cost(x_, u_opt_, run_time.toSec());
    // ROS_INFO_STREAM("stage cost: " << stage_cost_.data);
  }

  if (stage_cost_publisher_.trylock()) {
    stage_cost_publisher_.msg_ = stage_cost_;
    stage_cost_publisher_.unlockAndPublish();
  }

  {
    run_time = run_time + period;
  }
}

void ManipulationController::update_position_reference(
    const ros::Duration& period) {
  position_desired_.head<7>() += u_opt_.head<7>() * period.toSec();
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
  // clang-format on

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(arm_torque_command_[i]);
  }
}

void ManipulationController::stopping(const ros::Time& time) {
  if (record_bag_) {
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

PLUGINLIB_EXPORT_CLASS(manipulation_panda::ManipulationController,
                       controller_interface::ControllerBase)
