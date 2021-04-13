/*!
 * @file    omav_velocity_control.cpp
 * @author  Matthias Studiger
 * @date    10.04.2021
 * @version 1.0
 * @brief   description
 */
#include "mppi_omav_velocity/controller_interface.h"
#include "mppi_omav_velocity/dynamics_ros.h"
#include "mppi_omav_velocity/ros_conversions.h"

#include <chrono>
#include <memory>
#include <ros/ros.h>
#include <vector>

using namespace omav_velocity;

int main(int argc, char **argv) {
  // Initialize Ros Node
  ros::init(argc, argv, "omav_control_node");
  ros::NodeHandle nh("~");

  // ros interface
  auto controller = OMAVControllerInterface(nh);
  ROS_INFO_STREAM("Controller Created");

  auto robot_description_raisim =
      nh.param<std::string>("/robot_description_raisim", "");
  ROS_INFO_STREAM("Robot Description Raisim Loaded");

  auto simulation = std::make_shared<OMAVVelocityDynamicsRos>(
      nh, robot_description_raisim, 0.015);
  ROS_INFO_STREAM("Simulation Initialized");
  // Implement all the publishers
  trajectory_msgs::MultiDOFJointTrajectory optimal_trajectory_command;
  ros::Publisher optimal_trajectory_publisher_ =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          "/command/optimal_trajectory_reference", 10);

  // set initial state
  observation_t x = observation_t::Zero(simulation->get_state_dimension());
  auto x0 = nh.param<std::vector<double>>("initial_configuration", {});
  for (size_t i = 0; i < x0.size(); i++)
    x(i) = x0[i];
  // Set nominal state
  observation_t x_nom = observation_t::Zero(simulation->get_state_dimension());

  ROS_INFO_STREAM("Resetting initial state to " << x.transpose());
  simulation->reset(x);

  // init control input
  mppi::DynamicsBase::input_t u;
  u = simulation->get_zero_input(x);

  bool static_optimization = nh.param<bool>("static_optimization", false);
  auto sim_dt = nh.param<double>("sim_dt", 0.015);
  double sim_time = 0.0;

  // init the controller
  bool ok = controller.init();
  if (!ok)
    throw std::runtime_error("Failed to initialize controller");

  // set the very first observation
  controller.set_observation(x, sim_time);

  // start controller
  bool sequential;
  nh.param<bool>("sequential", sequential, false);
  if (!sequential)
    controller.start();

  // do some timing
  double elapsed;
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  // Remove before push

  while (ros::ok()) {
    start = std::chrono::steady_clock::now();
    if (sequential) {
      controller.update_reference();
      controller.set_observation(x, sim_time);
      controller.update_policy();
      controller.get_input_state(x, x_nom, u, sim_time);
      controller.publish_ros_default();
      // Additional publisher for additional visualization
      // controller.publish_optimal_rollout();
      // controller.publish_trajectories();

    } else {
      controller.set_observation(x, sim_time);
      controller.get_input_state(x, x_nom, u, sim_time);
    }
    if (!static_optimization) {
      x = simulation->step(u, sim_dt);
      simulation->publish_ros();
      sim_time += sim_dt;
    }

    // ros::Duration(0.1).sleep();

    // Conversation of the command and publication
    observation_array_t xx_opt;
    input_array_t uu_opt;
    controller.get_controller()->get_optimal_rollout(xx_opt, uu_opt);
    omav_velocity::conversions::to_trajectory_msg(xx_opt,
                                                  optimal_trajectory_command);
    optimal_trajectory_publisher_.publish(optimal_trajectory_command);

    end = std::chrono::steady_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
                  .count() /
              1000.0;

    if (sim_dt - elapsed > 0)
      ros::Duration(sim_dt - elapsed).sleep();
    else
      ROS_INFO_STREAM_THROTTLE(
          3.0, "Slower than real-time: " << elapsed / sim_dt << "x slower.");
    simulation->publish_ros();
    ros::spinOnce();
  }
}
