//
// Created by giuseppe on 22.01.21.
//

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "mppi_manipulation/controller_interface.h"
#include "mppi_manipulation/dynamics_ros.h"
#include <manipulation_msgs/State.h>

#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_example_controllers/JointTorqueComparison.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>

#include <manipulation_msgs/State.h>

namespace manipulation {

class RoyalPandaControllerRos
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaModelInterface, hardware_interface::EffortJointInterface,
          franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 private:
  bool init_parameters(ros::NodeHandle& node_handle);
  bool init_ros(ros::NodeHandle& node_handle);
  bool init_model(ros::NodeHandle& node_handle);
  bool init_interfaces(hardware_interface::RobotHW* robot_hw);

  void state_callback(const manipulation_msgs::StateConstPtr& state_msg);
  void stop_robot();
  void run_model();

  // Saturation
  std::array<double, 7> saturateTorqueRate(
      const std::array<double, 7>& tau_d_calculated,
      const std::array<double, 7>& tau_J_d);  // NOLINT (readability-identifier-naming)

 private:
  bool debug_;
  bool stopped_;

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  static constexpr double kDeltaTauMax{1.0};

  std::string arm_id_;
  std::vector<std::string> joint_names_;
  std::vector<double> k_gains_;
  std::vector<double> d_gains_;
  double coriolis_factor_{1.0};
  std::array<double, 7> dq_filtered_;
  franka::RobotState robot_state_;

  franka_hw::TriggerRate base_trigger_{50.0};
  franka_hw::TriggerRate rate_trigger_{1.0};
  std::array<double, 7> last_tau_d_{};
  realtime_tools::RealtimePublisher<franka_example_controllers::JointTorqueComparison>
      torques_publisher_;

  std::string base_twist_topic_;
  realtime_tools::RealtimePublisher<geometry_msgs::Twist> base_twist_publisher_;
  realtime_tools::RealtimePublisher<manipulation_msgs::State> nominal_state_publisher_;

  bool started_;
  std::unique_ptr<manipulation::PandaControllerInterface> man_interface_;

  Eigen::VectorXd x_;
  Eigen::VectorXd u_;

  Eigen::VectorXd x_nom_;

  manipulation_msgs::State x_ros_;
  manipulation_msgs::State x_nom_ros_;

  bool state_received_;
  bool state_ok_;
  double state_last_receipt_time_;
  std::string state_topic_;
  ros::Subscriber state_subscriber_;

  // debug
  bool fixed_base_;
  double start_time_;
  const double frequency_ = 0.5;
  double amplitude_ = 0.2; 

  std::array<double, 7> qd_;

  Eigen::Vector3d base_gains_;
  double base_filter_alpha_;

  std::shared_mutex input_mutex_;
  Eigen::VectorXd x_model_;
  manipulation_msgs::State x_model_ros_;
  realtime_tools::RealtimePublisher<manipulation_msgs::State> x_model_publisher_;
  std::thread model_thread_;
  std::unique_ptr<manipulation::ManipulatorDynamicsRos> model_;
};

}  // namespace manipulation