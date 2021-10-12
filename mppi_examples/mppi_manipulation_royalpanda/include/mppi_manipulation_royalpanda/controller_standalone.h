//
// Created by giuseppe on 22.01.21.
//

#pragma once

#include <memory>
#include <string>
#include <vector>

// clang-format off
#include <mppi_manipulation/controller_interface.h>
#include <mppi_manipulation/constraints/safety_filter.h>
// clang-format on

#include <mppi_manipulation/energy_tank.h>
#include <mppi_manipulation/params/filter_params.h>
#include <mppi_manipulation/params/gains.h>

#include <manipulation_msgs/InputState.h>
#include <manipulation_msgs/State.h>
#include "mppi_manipulation/dynamics_ros.h"

#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <geometry_msgs/TwistStamped.h>
#include <manipulation_msgs/State.h>
#include <mppi_ros/Rollout.h>
#include <mppi_ros/conversions.h>
#include <rosbag/bag.h>  
#include <std_msgs/Float64MultiArray.h>

namespace manipulation_royalpanda {

class ManipulationController{
 public:

  template <class T>
  using RTPublisher = realtime_tools::RealtimePublisher<T>;

  ManipulationController() = default;

  bool init(ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) ;
  void starting(const ros::Time&) ;
  void update(const ros::Time&, const ros::Duration& period) ;
  void stopping(const ros::Time& time) ;

 private:
  bool init_parameters(ros::NodeHandle& node_handle);

  void init_ros(ros::NodeHandle& node_handle);

  void state_callback(const manipulation_msgs::StateConstPtr& state_msg);

  static void getRotationMatrix(Eigen::Matrix3d& R, double theta);

 private:
  double start_time_;
  bool started_;
  bool sequential_;
  bool state_ok_;
  bool state_received_;
  bool fixed_base_;
  manipulation::PIDGains gains_;

  static constexpr double delta_tau_max_{1.0};

  std::string arm_id_;
  std::string state_topic_;
  std::string nominal_state_topic_;
  std::string base_twist_topic_;
  std::vector<std::string> joint_names_;

  ros::Subscriber state_subscriber_;
  std::unique_ptr<manipulation::PandaControllerInterface> man_interface_;

  std::mutex observation_mutex_;
  double observation_time_;
  Eigen::VectorXd x_;
  Eigen::VectorXd xfirst_;  // debug
  Eigen::VectorXd u_;
  Eigen::VectorXd u_opt_;
  Eigen::VectorXd x_nom_;
  Eigen::Matrix3d R_world_base;

  manipulation_msgs::State x_nom_ros_;
};
}  // namespace manipulation_royalpanda