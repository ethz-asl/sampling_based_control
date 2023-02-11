/*
 * Copyright 2021 Maximilian Brunner, ASL, ETH Zurich, Switzerland
 *
 * This code can be used only for academic, non-commercial use.
 * This code cannot be redistributed under any license, open source or
 * otherwise.
 *
 */

#ifndef OMAV_MPPI_INTERACTION_CONTROL_NODE_H
#define OMAV_MPPI_INTERACTION_CONTROL_NODE_H

#include <mppi_omav_interaction/controller_interface.h>
#include <mppi_omav_interaction/omav_interaction_common.h>

#include <mppi_omav_interaction/MPPIOmavCostShelfConfig.h>
#include <mppi_omav_interaction/MPPIOmavCostValveConfig.h>
#include <mppi_omav_interaction/MPPIOmavSettingsConfig.h>
// ros
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

namespace omav_interaction {

// Simulation timestep that is used inside the mppi simulation.
constexpr double kSim_dt = 0.015;
constexpr int64_t kSim_dt_ns = 15e6;
// Rate at which the controller checks for new odom updates and - if available -
// updates the mppi state (currently deprecated, only used with timed control)
// const double kControl_rate = 250.0;

class InteractionControlNode {
 public:
  InteractionControlNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  ~InteractionControlNode();

 private:
  bool InitializeNodeParams();
  bool computeCommand(const ros::Time& t_now);
  // void TimedCommandCallback(const ros::TimerEvent& e);

  bool getState(observation_t& x);
  bool getState(observation_t& x, double& timestamp, bool& is_new);
  bool getTargetStateFromTrajectory();
  bool setTarget(const trajectory_msgs::MultiDOFJointTrajectoryPoint&
                     trajectory_msg_point);
  bool setTarget(const mav_msgs::EigenTrajectoryPoint& target_state);

  void initializeSubscribers();
  bool initialize_integrators(observation_t& x);
  void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
  void objectStateCallback(const sensor_msgs::JointState& object_msg);
  void objectPoseCallback(const geometry_msgs::Pose& pose_msg);

  void costShelfParamCallback(mppi_omav_interaction::MPPIOmavCostShelfConfig& config,
                         uint32_t level);
  void costValveParamCallback(
      mppi_omav_interaction::MPPIOmavCostValveConfig& config, uint32_t level);
  void mppiSettingsParamCallback(
      mppi_omav_interaction::MPPIOmavSettingsConfig& config, uint32_t level);
  template <class T>
  void getParam(const ros::NodeHandle& nh, const std::string& id, T& var,
                const T& val_def) const;

  ros::NodeHandle nh_, private_nh_;
  // ros::Timer timed_command_;

  bool running_rotors_ = true;
  // bool sequential_ = false;
  // double sim_time_ = 0.0;

  bool controller_running_ = false;

  ros::Subscriber odometry_sub_;
  ros::Subscriber object_state_sub_;
  ros::Subscriber object_pose_subscriber_;

  observation_t state_;

  OMAVControllerInterface controller_;
  // std::shared_ptr<OMAVVelocityDynamicsRos> simulation_;

  // Odometry Variable
  mav_msgs::EigenOdometry current_odometry_;
  bool object_valid_ = false;
  bool odometry_valid_ = false;
  bool trajectory_available_ = false;

  trajectory_msgs::MultiDOFJointTrajectory current_trajectory_;
  mav_msgs::EigenTrajectoryPoint target_state_;
  Eigen::Vector2d object_state_;
  ros::Time object_state_time_;
  Eigen::Matrix<double, 7, 1> object_pose_;

  // Dynamics Reconfigure
  dynamic_reconfigure::Server<mppi_omav_interaction::MPPIOmavCostShelfConfig>
      cost_shelf_param_server_;
  dynamic_reconfigure::Server<mppi_omav_interaction::MPPIOmavCostValveConfig>
      cost_valve_param_server_;
  dynamic_reconfigure::Server<mppi_omav_interaction::MPPIOmavSettingsConfig>
      mppiSettingsParamServer_;

  OMAVInteractionCostShelfParam cost_shelf_params_;
  OMAVInteractionCostValveParam cost_valve_params_;
};
}  // namespace omav_interaction
#endif  // OMAV_MPPI_INTERACTION_CONTROL_NODE_H
