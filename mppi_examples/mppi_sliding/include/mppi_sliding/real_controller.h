//
// Created by giuseppe on 22.01.21.
//

#pragma once

#include <memory>
#include <string>
#include <vector>

// clang-format off
#include <mppi_sliding/controller_interface.h>
//#include <mppi_manipulation/constraints/safety_filter.h>
// clang-format on

//#include <mppi_manipulation/energy_tank.h>
//#include <mppi_manipulation/params/filter_params.h>
#include <mppi_sliding/params/gains.h>

#include <manipulation_msgs/InputState.h>
#include <manipulation_msgs/State.h>
#include <mppi_sliding/dynamics_ros.h>

#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>

#include <geometry_msgs/TwistStamped.h>
#include <mppi_ros/Rollout.h>
#include <mppi_ros/conversions.h>
#include <rosbag/bag.h>
#include <std_msgs/Float64MultiArray.h>

namespace manipulation_panda {

class ManipulationController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::VelocityJointInterface,
          hardware_interface::EffortJointInterface,
          franka_hw::FrankaModelInterface, franka_hw::FrankaStateInterface> {
 public:
  // explicit controller to allow for a missing hardware interface
  using BASE = controller_interface::MultiInterfaceController<
      hardware_interface::VelocityJointInterface,
      hardware_interface::EffortJointInterface, franka_hw::FrankaModelInterface,
      franka_hw::FrankaStateInterface>;

  template <class T>
  using RTPublisher = realtime_tools::RealtimePublisher<T>;

  ManipulationController() : BASE(true){};

  bool init(hardware_interface::RobotHW* robot_hw,
            ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 private:
  bool init_parameters(ros::NodeHandle& node_handle);
  bool init_interfaces(hardware_interface::RobotHW* robot_hw);
  void init_ros(ros::NodeHandle& node_handle);
  void init_env();
  void state_callback(const manipulation_msgs::StateConstPtr& state_msg);
<<<<<<< HEAD

  // Apply the safety filter to the optimized velocity
  void enforce_constraints(const ros::Duration& period);
=======
>>>>>>> d8de59e28d867005f93ea195df8a82ff10706baf

  // Update the desired position reference
  void update_position_reference(const ros::Duration& period);

  // Convert velocity command to effort command using a PI controller
  void send_command_arm(const ros::Duration& period);

  // Directy send velocity commands to the base (via topic on hardware)
  void send_command_base(const ros::Duration& period);

  // Saturation
  void saturateTorqueRate(const std::array<double, 7>& tau_d_calculated,
                          const std::array<double, 7>& tau_J_d,
                          Eigen::Matrix<double, 7, 1>& torque_cmd);

  static void getRotationMatrix(Eigen::Matrix3d& R, double theta);

 private:
  bool started_;
  bool sequential_;
  bool state_ok_;
  bool state_received_;

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  bool has_base_handles_;

  manipulation::PIDGains gains_;

  static constexpr double delta_tau_max_{10.0};
  franka_hw::TriggerRate base_trigger_{50.0};

  std::string arm_id_;
  std::string state_topic_;
  std::string nominal_state_topic_;
  std::string table_state_topic_;
  std::vector<std::string> joint_names_;

  ros::Subscriber state_subscriber_;
  RTPublisher<manipulation_msgs::State> nominal_state_publisher_;
  RTPublisher<sensor_msgs::JointState> table_state_publisher_;

  std::unique_ptr<manipulation::PandaControllerInterface> man_interface_;

  std::mutex observation_mutex_;
  double observation_time_;
  Eigen::VectorXd x_;
  Eigen::VectorXd u_;
  Eigen::VectorXd u_opt_;
  Eigen::VectorXd x_nom_;
  Eigen::VectorXd position_initial_;
  Eigen::VectorXd position_measured_;
  Eigen::VectorXd position_desired_;
  Eigen::VectorXd velocity_measured_;
  Eigen::VectorXd velocity_filtered_;
  Eigen::VectorXd max_position_error_;
  franka::RobotState robot_state_;

  manipulation_msgs::State x_ros_;
  manipulation_msgs::State x_nom_ros_;

  Eigen::Matrix<double, 7, 1> arm_torque_command_;

<<<<<<< HEAD
  // safety filter
  bool apply_filter_;
  bool apply_filter_to_rollouts_;
  // manipulation::EnergyTank energy_tank_;
  // manipulation::FilterParams safety_filter_params_;
  // std::unique_ptr<manipulation::PandaMobileSafetyFilter> safety_filter_;

=======
>>>>>>> d8de59e28d867005f93ea195df8a82ff10706baf
  // metrics
  double stage_cost_;

  // optimal rollout info
  bool publish_rollout_;
  mppi::Rollout optimal_rollout_;
  mppi_ros::Rollout optimal_rollout_ros_;
  std_msgs::Float64MultiArray u_curr_ros_;
  rosbag::Bag bag_;
  std::string bag_path_;
  bool record_bag_;

  // logging
  int log_counter_ = 0;
  int log_every_steps_;
  double opt_time_;
};
}  // namespace manipulation_panda