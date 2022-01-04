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

#include <mppi_omav_interaction/omav_trajectory_generator.h>
#include <mppi_omav_interaction/controller_interface.h>

// ros
#include <ros/ros.h>

namespace omav_interaction {

// Simulation timestep that is used inside the mppi simulation.
const double kSim_dt = 0.015;
// Rate at which the controller checks for new odom updates and - if available - updates the mppi state
const double kControl_rate = 250.0;

class InteractionControlNode {
 public:
  InteractionControlNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  ~InteractionControlNode();

 private:
  bool InitializeNodeParams();
  bool InitializeControllerParams();
  bool computeCommand();
  void TimedCommandCallback(const ros::TimerEvent& e);
  template <class T>
  void getParam(const ros::NodeHandle& nh, const std::string& id, T& var,
                const T& val_def) const;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Timer timed_command_;

  bool running_rotors_ = true;
  bool sequential_ = false;
  double sim_time_ = 0.0;
  int index_temp_ = 0;
  // double t_elapsed_;

  bool controller_running_ = false;

  mppi::DynamicsBase::input_t input_;
  observation_t state_nom_;
  observation_t state_;

  OMAVControllerInterface controller_;
  std::shared_ptr<OmavTrajectoryGenerator> omav_trajectory_node_;
  std::shared_ptr<OMAVVelocityDynamicsRos> simulation_;
};
}  // namespace omav_interaction
#endif  // OMAV_MPPI_INTERACTION_CONTROL_NODE_H
