/*!
 * @file     controller_interface.h
 * @author   Matthias Studiger
 * @date     10.04.2020
 * @version  1.0
 * @brief    description
 */

#pragma once

#include "mppi_omav_velocity/ros_conversions.h"
#include <geometry_msgs/PoseStamped.h>
#include <mppi_ros/controller_interface.h>

#include <tf/transform_broadcaster.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <memory>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>

namespace omav_velocity {

class OMAVControllerInterface : public mppi_ros::ControllerRos {
public:
  explicit OMAVControllerInterface(ros::NodeHandle &nh,
                                   ros::NodeHandle &nh_public)
      : ControllerRos(nh, nh_public) {}
  ~OMAVControllerInterface() = default;

  bool init_ros() override;
  void publish_ros() override;
  void publish_trajectories() override;
  void publish_optimal_rollout() override;
  bool update_reference() override;
  bool set_reference(const observation_t &x);

private:
  bool set_controller(std::shared_ptr<mppi::PathIntegral> &controller) override;
  void desired_pose_callback(const geometry_msgs::PoseStampedConstPtr &msg);
  void indicator_callback(const std_msgs::Int64 &msg);
  void mode_callback(const std_msgs::Int64ConstPtr &msg);
  void obstaclexCallback(const std_msgs::Float32 &msg);
  void obstacleyCallback(const std_msgs::Float32 &msg);
  void publishBoolCallback(const std_msgs::Bool &publish_bool);
  void object_reference_callback(const geometry_msgs::PoseStampedConstPtr &msg);
  void publish_trajectory(const mppi::observation_array_t &x_opt);

public:
  mppi::SolverConfig config_;

private:
  bool reference_set_ = false;
  bool publish_bool_ = false;

  ros::Publisher cmd_multi_dof_joint_trajectory_pub_;

  trajectory_msgs::MultiDOFJointTrajectory current_trajectory_msg_;

  ros::Subscriber reference_subscriber_;
  ros::Subscriber indicator_subscriber_;
  ros::Subscriber obstacle_x_sub_;
  ros::Subscriber obstacle_y_sub_;
  ros::Subscriber publish_bool_sub_;
  ros::Subscriber object_reference_subscriber_;
  ros::Subscriber mode_subscriber_;

  ros::Publisher object_state_publisher_;

  sensor_msgs::JointState object_state_;

  std::string robot_description_raisim_;
  std::string object_description_raisim_;

  std::mutex reference_mutex_;
  mppi::reference_trajectory_t ref_;

  observation_array_t xx_opt;
  input_array_t uu_opt;
};
} // namespace omav_velocity
