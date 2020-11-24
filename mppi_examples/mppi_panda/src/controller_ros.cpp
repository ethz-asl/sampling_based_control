/*!
 * @file     controller_ros.cpp
 * @author   Giuseppe Rizzi
 * @date     10.09.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_panda/controller_ros.h"

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <franka_example_controllers/compliance_paramConfig.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <cmath>
#include <functional>

namespace panda {

template <class StateInterface, class StateHandle>
bool PandaControllerRosBase<StateInterface, StateHandle>::init(
    hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle,
    ros::NodeHandle& ctrl_handle) {
  std::string robot_description;
  if (!node_handle.getParam("/robot_description", robot_description)) {
    ROS_ERROR_STREAM("Can't read robot description.");
    return false;
  }

  bool added = addStateHandles(robot_hw);
  if (!added) {
    return added;
  }

  robot_ = std::make_shared<rc::RobotWrapper>();
  robot_->initFromXml(robot_description);

  auto kp = ctrl_handle.param<std::vector<double>>("kp", {});
  auto kd = ctrl_handle.param<std::vector<double>>("kd", {});

  if (kp.size() != 7) {
    ROS_ERROR_STREAM("The proportional gain has the wrong size: " << kp.size()
                                                                  << "!=" << 7);
    return false;
  }
  if (kd.size() != 7) {
    ROS_ERROR_STREAM("The proportional gain has the wrong size: " << kp.size()
                                                                  << "!=" << 7);
    return false;
  }
  Kp_.setZero();
  Kd_.setZero();
  for (size_t i = 0; i < 7; i++) {
    Kp_(i, i) = kp[i];
    Kd_(i, i) = kd[i];
  }
  ROS_INFO_STREAM("P gain:\n" << Kp_);
  ROS_INFO_STREAM("D gain:\n" << Kd_);

  x_ = mppi::observation_t::Zero(PandaDim::STATE_DIMENSION);
  u_ = mppi::observation_t::Zero(PandaDim::INPUT_DIMENSION);
  controller_ = std::make_unique<PandaControllerInterface>(ctrl_handle);
  if (!controller_->init()){
    ROS_ERROR("Failed to initialized MPPI controller");
    return false;
  }
  started_ = false;

  ee_pose_publisher_ = std::make_unique<
      realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>>(
      node_handle, "/end_effector", 10);
  return true;
}

bool PandaControllerRos::addStateHandles(
    hardware_interface::RobotHW* robot_hw) {
  auto state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("Can't get franka state interface");
    return false;
  }

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle;
  try {
    state_handle = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle("panda_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("Excepion getting franka state handle: " << ex.what());
    return false;
  }

  const franka::RobotState& state = state_handle->getRobotState();
  for (size_t i = 0; i < 7; i++) {
    hardware_interface::JointStateHandle joint_state_handle =
        hardware_interface::JointStateHandle(joint_names_[i], &state.q[i],
                                             &state.dq[i], &state.tau_J[i]);
    state_handles_.push_back(joint_state_handle);
  }

  auto velocity_joint_interface =
      robot_hw->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface == nullptr) {
    ROS_ERROR_STREAM("Error getting velocity joint interface.");
    return false;
  }
  for (auto& joint_name : joint_names_) {
    joint_handles_.push_back(velocity_joint_interface->getHandle(joint_name));
  }
  return true;
}

bool PandaControllerRosSim::addStateHandles(
    hardware_interface::RobotHW* robot_hw) {
  auto state_interface =
      robot_hw->get<hardware_interface::JointStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("Can't get franka state interface");
    return false;
  }

  auto effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("Error getting effort joint interface.");
    return false;
  }
  for (auto& joint_name : joint_names_) {
    state_handles_.push_back(state_interface->getHandle(joint_name));
    joint_handles_.push_back(effort_joint_interface->getHandle(joint_name));
  }
  return true;
}

template <class SI, class SH>
void PandaControllerRosBase<SI, SH>::starting(const ros::Time& time) {
  if (started_) return;

  Eigen::VectorXd q = getJointPositions();
  Eigen::VectorXd v = getJointVelocities();
  x_.head<7>() = q;
  x_.tail<7>() = v;
  controller_->set_observation(x_, time.toSec());
  controller_->start();
  started_ = true;
}

void PandaControllerRosSim::update(const ros::Time& time,
                                   const ros::Duration& period) {
  if (period.toSec() > 0.001){
    ROS_ERROR_STREAM_THROTTLE(1.0,"Last controller call lasted: " << period.toSec());
  }

  Eigen::VectorXd q = getJointPositions();
  Eigen::VectorXd v = getJointVelocities();
  x_.head<7>() = q;
  x_.tail<7>() = v;
  controller_->set_observation(x_, time.toSec());

  robot_->updateState(q, v);
  robot_->computeAllTerms();
  tau_ = robot_->getNonLinearTerms();

  controller_->get_input_state(x_, x_des_, u_, time.toSec());
  tau_ += Kp_ * (x_des_.head<7>() - q) + Kd_ * (u_ - v);

  for (size_t i = 0; i < 7; i++) {
    joint_handles_[i].setCommand(tau_(i));
  }
  publish_ros();
}

void PandaControllerRos::update(const ros::Time& time,
                                const ros::Duration& period) {
  x_.head<7>() = getJointPositions();
  x_.tail<7>() = getJointVelocities();
  controller_->set_observation(x_, time.toSec());
  controller_->get_input(x_, u_, time.toSec());
  for (size_t i = 0; i < 7; i++) {
    joint_handles_[i].setCommand(u_(i));
  }
  robot_->updateState(x_.head<7>(), x_.tail<7>());
  publish_ros();
}

template <class SI, class SH>
Eigen::VectorXd PandaControllerRosBase<SI, SH>::getJointVelocities() const {
  Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(9);
  for (size_t i = 0; i < 7; i++) {
    joint_velocities(i) = state_handles_[i].getVelocity();
  }
  return joint_velocities;
}

template <class SI, class SH>
Eigen::VectorXd PandaControllerRosBase<SI, SH>::getJointPositions() const {
  Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(9);
  for (size_t i = 0; i < 7; i++) {
    joint_positions(i) = state_handles_[i].getPosition();
  }
  return joint_positions;
}

template <class SI, class SH>
void PandaControllerRosBase<SI, SH>::publish_ros() {
  // for debugging
  static std::string ee_frame = "panda_hand";
  if (ee_pose_publisher_->trylock()) {
    auto current_pose_ = robot_->getFramePlacement(ee_frame);
    ee_pose_publisher_->msg_.header.stamp = ros::Time::now();
    ee_pose_publisher_->msg_.header.frame_id = "world";
    ee_pose_publisher_->msg_.pose.position.x = current_pose_.translation()(0);
    ee_pose_publisher_->msg_.pose.position.y = current_pose_.translation()(1);
    ee_pose_publisher_->msg_.pose.position.z = current_pose_.translation()(2);
    Eigen::Quaterniond q(current_pose_.rotation());
    ee_pose_publisher_->msg_.pose.orientation.x = q.x();
    ee_pose_publisher_->msg_.pose.orientation.y = q.y();
    ee_pose_publisher_->msg_.pose.orientation.z = q.z();
    ee_pose_publisher_->msg_.pose.orientation.w = q.w();
    ee_pose_publisher_->unlockAndPublish();
  }
}
}  // namespace panda

PLUGINLIB_EXPORT_CLASS(panda::PandaControllerRos,
                       controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(panda::PandaControllerRosSim,
                       controller_interface::ControllerBase)
