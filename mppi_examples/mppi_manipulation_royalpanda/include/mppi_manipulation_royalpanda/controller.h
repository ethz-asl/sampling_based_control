//
// Created by giuseppe on 22.01.21.
//

#pragma once

#include <memory>
#include <string>
#include <vector>

// clang-format off  -> this header must come first always
#include <mppi_manipulation/controller_interface.h>
// clang-format on

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

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>

#include <geometry_msgs/TwistStamped.h>
#include <manipulation_msgs/State.h>
#include <mppi_ros/Rollout.h>
#include <mppi_ros/conversions.h>
#include <rosbag/bag.h>  
#include <std_msgs/Float64MultiArray.h>

namespace manipulation_royalpanda {

//clang-format off
class ManipulationController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::VelocityJointInterface,
          hardware_interface::EffortJointInterface,
          franka_hw::FrankaModelInterface, 
          franka_hw::FrankaStateInterface> {
 public:
  // explicit controller to allow for a missing hardware interface
  using BASE = controller_interface::MultiInterfaceController<
      hardware_interface::VelocityJointInterface,
      hardware_interface::EffortJointInterface,
      franka_hw::FrankaModelInterface, 
      franka_hw::FrankaStateInterface>;
  // clang-format on

  template <class T>
  using RTPublisher = realtime_tools::RealtimePublisher<T>;

  ManipulationController() : BASE(true){};

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 private:
  bool init_parameters(ros::NodeHandle& node_handle);
  bool init_interfaces(hardware_interface::RobotHW* robot_hw);
  void init_ros(ros::NodeHandle& node_handle);

  void state_callback(const manipulation_msgs::StateConstPtr& state_msg);

  /**
   * @brief Update the position reference, integrating the input velocity command
   */
  void update_position_reference(const ros::Duration& period);

  /**
   * @brief Convert the reference velocity and position in torque commands
   * that are sent to the robot arm through the EffortJointInterface
   */
  void send_command_arm(const ros::Duration& period);

  /**
   * @brief Send a velocity command to the base (only velocity controlled)
   * Note: the base accepts command in the base frame while our formulation 
   * assumes that x_dot and y_dot are aligned in the global frame. 
   */
  void send_command_base(const ros::Duration& period);

  /**
   * @brief Publish additional debugging information to ros
   * Note: use RealtimePublisher to avoid slowdown http://wiki.ros.org/realtime_tools
   */
  void publish_ros();

  /**
   * @brief This function is copied from some of the franka ros controller.
   * Apparently franka requires torque_rate below a certain threshold. This function
   * is ensuring that the torque does not change "too rapidly" 
   */
  void saturateTorqueRate(const std::array<double, 7>& tau_d_calculated,
                          const std::array<double, 7>& tau_J_d,
                          Eigen::VectorXd& torque_cmd);

  /**
   * @brief Get the Rotation Matrix object from a single yaw angle
   */
  static void getRotationMatrix(Eigen::Matrix3d& R, double theta);

 private:
  bool debug_ = false;
  bool simulation_;
  bool started_;
  bool started_twice_;
  bool sequential_;
  bool state_ok_;
  bool state_received_;

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  bool has_base_handles_;
  std::vector<std::string> base_joint_names_;
  std::vector<hardware_interface::JointHandle> base_joint_handles_;

  bool fixed_base_;
  manipulation::PIDGains gains_;
  std::vector<double> base_cmd_threshold_;

  static constexpr double delta_tau_max_{1.0};
  franka_hw::TriggerRate base_trigger_{50.0};

  std::string arm_id_;
  std::string state_topic_;
  std::string nominal_state_topic_;
  std::string base_twist_topic_;
  std::vector<std::string> joint_names_;

  ros::Subscriber state_subscriber_;
  RTPublisher<geometry_msgs::Twist> base_twist_publisher_;
  RTPublisher<geometry_msgs::Twist> world_twist_publisher_;
  RTPublisher<manipulation_msgs::State> nominal_state_publisher_;
  RTPublisher<std_msgs::Float64MultiArray> input_publisher_;
  RTPublisher<std_msgs::Float64MultiArray> position_desired_publisher_;
  
  // the actual sampling-based controller
  std::unique_ptr<manipulation::PandaControllerInterface> man_interface_;

  std::mutex observation_mutex_;
  Eigen::VectorXd x_;
  Eigen::VectorXd u_;
  Eigen::VectorXd x_nom_;
  
  Eigen::VectorXd position_initial_;
  Eigen::VectorXd position_measured_;
  Eigen::VectorXd position_desired_;
  Eigen::VectorXd position_error_max_;
  Eigen::VectorXd velocity_measured_;
  Eigen::VectorXd velocity_filtered_;
  Eigen::VectorXd torque_desired_;
  
  Eigen::Matrix3d R_world_base_;
  franka::RobotState robot_state_;

  manipulation_msgs::State x_ros_;
  manipulation_msgs::State x_nom_ros_;

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
  bool logging_;
  std::string log_file_path_;
  int log_counter_ = 0;
  int log_every_steps_;
  double opt_time_;

  // time
  double start_time_;
  double current_time_;
  double observation_time_;
  double measurement_time_;  // the ROS based time measurement
};
}  // namespace manipulation_royalpanda