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
#include <mppi_omav_interaction/cost.h>
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

  bool update_cost_param(const OMAVInteractionCostParam &cost_param);

  bool set_reference(const observation_t &x);

  bool set_rqt_reference(const Eigen::VectorXd &rqt_ref);

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
  std::shared_ptr<OMAVInteractionCost> cost_;

private:
  bool reference_set_ = false;

  observation_t x_0_temp;

  OMAVInteractionCostParam cost_param_;

  ros::Publisher cmd_multi_dof_joint_trajectory_pub_;
  ros::Publisher cost_publisher_;
  ros::Publisher torque_publisher_;

  trajectory_msgs::MultiDOFJointTrajectory current_trajectory_msg_;

  ros::Subscriber reference_subscriber_;
  ros::Subscriber mode_subscriber_;
  ros::Subscriber object_reference_subscriber_;

  std::string robot_description_raisim_;
  std::string robot_description_pinocchio_;
  std::string object_description_raisim_;

  std::mutex reference_mutex_;
  mppi::reference_trajectory_t ref_;

  observation_array_t xx_opt;
  input_array_t uu_opt;

  mppi_pinocchio::RobotModel robot_model_;
  mppi_pinocchio::RobotModel object_model_;
  Eigen::Vector3d tip_lin_velocity_;
  Eigen::Matrix<double, 6, 1> tip_velocity_;
  Eigen::Vector3d torque_;
  Eigen::Vector3d hook_handle_vector_;
  float distance_hook_handle_;
  Eigen::Vector3d force_normed_;
  Eigen::Vector3d com_hook_;
  float torque_angle_;
  // cost floats to publish
  float velocity_cost_;
  float handle_hook_cost_;
  float delta_pose_cost_;
  float power_cost_;
  float torque_cost_;
};
} // namespace omav_interaction
