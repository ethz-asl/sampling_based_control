//
// Created by giuseppe on 22.01.21.
//

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <robot_control/modeling/robot_wrapper.h>
#include "mppi_manipulation/controller_interface.h"

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

#include <franka_example_controllers/JointTorqueComparison.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>

#include <geometry_msgs/TwistStamped.h>
#include <manipulation_msgs/State.h>

namespace royalpanda {

class RoyalPandaControllerRos
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaModelInterface,
          hardware_interface::EffortJointInterface,
          franka_hw::FrankaStateInterface> {
 public:
  // explicit controller to allow for a missing hardware interface
  using BASE = controller_interface::MultiInterfaceController<
      franka_hw::FrankaModelInterface, hardware_interface::EffortJointInterface,
      franka_hw::FrankaStateInterface>;
  RoyalPandaControllerRos() : BASE(true){};

  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 private:
  bool init_parameters(ros::NodeHandle& node_handle);
  bool init_ros(ros::NodeHandle& node_handle);

  bool init_common_interfaces(hardware_interface::RobotHW* robot_hw);
  bool init_franka_interfaces(hardware_interface::RobotHW* robot_hw);

  void state_callback(const manipulation_msgs::StateConstPtr& state_msg);

  void send_command_arm(const ros::Duration& period);
  void send_command_base(const ros::Duration& period);
  void send_command_base_from_position(const ros::Duration& period);

  // Saturation
  std::array<double, 7> saturateTorqueRate(
      const std::array<double, 7>& tau_d_calculated,
      const std::array<double, 7>&
          tau_J_d);  // NOLINT (readability-identifier-naming)

  // simulation subscribers to external controller
  void input_state_callback(const manipulation_msgs::InputStateConstPtr& msg);

 private:
  bool sim_;

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  static constexpr double kDeltaTauMax{1.0};

  std::string arm_id_;
  std::vector<std::string> joint_names_;
  std::vector<double> i_gains_;
  std::vector<double> d_gains_;
  double coriolis_factor_{1.0};
  std::array<double, 7> dq_filtered_;
  franka::RobotState robot_state_;

  franka_hw::TriggerRate base_trigger_{50.0};
  franka_hw::TriggerRate rate_trigger_{1.0};
  std::array<double, 7> last_tau_d_{};
  realtime_tools::RealtimePublisher<
      franka_example_controllers::JointTorqueComparison>
      torques_publisher_;

  std::string base_twist_topic_;
  realtime_tools::RealtimePublisher<geometry_msgs::Twist> base_twist_publisher_;
  realtime_tools::RealtimePublisher<manipulation_msgs::State>
      nominal_state_publisher_;

  bool started_;
  std::unique_ptr<manipulation::PandaControllerInterface> man_interface_;

  Eigen::VectorXd x_;
  Eigen::VectorXd u_;

  Eigen::VectorXd x_nom_;

  manipulation_msgs::State x_ros_;
  manipulation_msgs::State x_nom_ros_;

  bool state_ok_;
  bool state_received_;
  double state_last_receipt_time_;
  std::string state_topic_;
  ros::Subscriber state_subscriber_;

  bool fixed_base_;
  double start_time_;

  std::array<double, 7> qd_;

  double base_filter_alpha_;

  geometry_msgs::TwistStamped twist_stamped_;
  ros::Publisher world_twist_publisher_;

  std::unique_ptr<rc::RobotWrapper> robot_model_;

  manipulation_msgs::InputState input_state_;
  ros::Subscriber input_state_subscriber_;

  Eigen::Vector3d base_K_gains_;
  Eigen::Vector3d base_I_gains_;
  double I_max_;

  Eigen::Vector3d twist_cmd_;
  Eigen::Vector3d base_position_error_;
  Eigen::Vector3d base_integral_error_;

  Eigen::Matrix<double, 7, 1> arm_integral_error_;
  double arm_I_max_;
};

}  // namespace royalpanda