/*!
 * @file     controller_interface.h
 * @author   Matthias Studiger
 * @date     10.04.2020
 * @version  1.0
 * @brief    description
 */

#pragma once

#include "mppi_omav_interaction/ros_conversions.h"
#include <geometry_msgs/Pose.h>
#include <mppi_ros/controller_interface.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <memory>
#include <std_msgs/Int64.h>

namespace omav_interaction {

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

  bool set_reset_reference(const observation_t &x);

  bool set_object_reference();

private:
  bool set_controller(std::shared_ptr<mppi::PathIntegral> &controller) override;

  void desired_pose_callback(const geometry_msgs::PoseStampedConstPtr &msg);

  void mode_callback(const std_msgs::Int64ConstPtr &msg);

  void object_reference_callback(const geometry_msgs::PoseStampedConstPtr &msg);

  void publish_trajectory(const mppi::observation_array_t &x_opt);

public:
  mppi::SolverConfig config_;

private:
  bool reference_set_ = false;

  observation_t x_0_temp;

  ros::Publisher cmd_multi_dof_joint_trajectory_pub_;

  trajectory_msgs::MultiDOFJointTrajectory current_trajectory_msg_;

  ros::Subscriber reference_subscriber_;
  ros::Subscriber mode_subscriber_;
  ros::Subscriber object_reference_subscriber_;

  std::mutex reference_mutex_;
  mppi::reference_trajectory_t ref_;

  observation_array_t xx_opt;
  input_array_t uu_opt;
};
} // namespace omav_velocity
