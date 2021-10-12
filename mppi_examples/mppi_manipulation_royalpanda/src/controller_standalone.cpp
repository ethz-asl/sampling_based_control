//
// Created by giuseppe on 22.01.21.
//

#include <mppi_manipulation_royalpanda/controller_standalone.h>
#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <ros/ros.h>
#include <signal_logger/signal_logger.hpp>

#include <manipulation_msgs/conversions.h>
#include <chrono>

using namespace manipulation;
using namespace manipulation_royalpanda;

bool ManipulationController::init(ros::NodeHandle& root_nh,
                                  ros::NodeHandle& controller_nh) {
  if (!init_parameters(controller_nh)) return false;
  init_ros(controller_nh);

  man_interface_ = std::make_unique<PandaControllerInterface>(controller_nh);
  if (!man_interface_->init()) {
    ROS_ERROR("Failed to initialized manipulation interface");
    return false;
  }
  ROS_INFO("Solver initialized correctly.");


  // we do not actively control the gripper
  x_.setZero(PandaDim::STATE_DIMENSION);
  xfirst_.setZero(PandaDim::STATE_DIMENSION);
  xfirst_.head<12>() << 0.943846, 1.35497, -1.99787, 0.170679, -0.75044, -0.261203, -2.25202, -0.0422951, 1.38016, 0.854492, 0.0405111, 0.0405111;
  x_nom_.setZero(PandaDim::STATE_DIMENSION);
  u_.setZero(PandaDim::INPUT_DIMENSION);
  
  ROS_INFO("Controller successfully initialized!");
  started_ = false;
  return true;
}

bool ManipulationController::init_parameters(ros::NodeHandle& node_handle) {
  if (!node_handle.getParam("state_topic", state_topic_)) {
    ROS_ERROR("state_topic not found");
    return false;
  }


  ROS_INFO("Parameters successfully initialized.");
  start_time_ = ros::Time::now().toSec();
  return true;
}

void ManipulationController::init_ros(ros::NodeHandle& nh) {
  state_subscriber_ = nh.subscribe(
     state_topic_, 1, &ManipulationController::state_callback, this);
}

void ManipulationController::state_callback(
    const manipulation_msgs::StateConstPtr& state_msg) {
  if (!started_) return;

  {
    // std::unique_lock<std::mutex> lock(observation_mutex_);
    // manipulation::conversions::msgToEigen(*state_msg, x_, observation_time_);
    // x_(STATE_DIMENSION - TORQUE_DIMENSION - 1) = 0.0; //energy_tank_.get_state();
    

    if (!state_received_) {
      // ROS_INFO("First call to state callback");
      // if (!man_interface_->init_reference_to_current_pose(x_,
      //                                                     observation_time_)) {
      //   ROS_WARN("Failed to set the controller reference to current state.");
      // }
      // //xfirst_ = x_;
      ROS_INFO_STREAM("Resetting the state always at the first state:\n"
                      << xfirst_.transpose());
    }
  } 
  state_received_ = true;
}

void ManipulationController::starting(const ros::Time& time) {
  state_received_ = false;
  if (started_) {
    ROS_INFO("Controller already started!");
    ROS_INFO_STREAM("Manipulation controller address: " << man_interface_.get());
    return;
  }

  // with sequential execution the optimization needs to be explicitly called
  // in the update function of the controller (not real-time safe)
  started_ = man_interface_->start();
  if (!started_) {
    ROS_ERROR("Failed  to start controller");
    return;
  }

  ROS_INFO_STREAM("Manipulation Controller address: " << man_interface_.get());
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

  man_interface_->set_observation(xfirst_, time.toSec() - start_time_);
  getRotationMatrix(R_world_base, xfirst_(2));
  man_interface_->get_input_state(xfirst_, x_nom_, u_, time.toSec() - start_time_); 
}

void ManipulationController::stopping(const ros::Time& time) {
  ROS_INFO("Stopping controller.");
}


void ManipulationController::getRotationMatrix(Eigen::Matrix3d& R,
                                               const double theta) {
  // clang-format off
  R << std::cos(theta), -std::sin(theta), 0.0,
       std::sin(theta), std::cos(theta), 0.0,
       0.0, 0.0, 1.0;
  // clang-format on
}
