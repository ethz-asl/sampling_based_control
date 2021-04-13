/*!
 * @file    omav_control.cpp
 * @author  Matthias Studiger
 * @date    18.03.2021
 * @version 1.0
 * @brief   description
 */
#include "mppi_omav_raisim/controller_interface.h"
#include "mppi_omav_raisim/dynamics_ros.h"
#include "mppi_omav_raisim/ros_conversions.h"

#include <chrono>
#include <memory>
#include <ros/ros.h>
#include <vector>

using namespace omav_raisim;

int main(int argc, char **argv) {
  // Initialize Ros Node
  ros::init(argc, argv, "omav_raisim_control_node");
  ros::NodeHandle nh("~");

  // ros interface
  auto controller = OMAVControllerInterface(nh);

  auto robot_description = nh.param<std::string>("/robot_description", "");
  auto robot_description_raisim =
      nh.param<std::string>("/robot_description_raisim", "");

  auto simulation = std::make_shared<OMAVRaisimDynamicsRos>(
      nh, robot_description_raisim, 0.015);
  // Implement all the publishers
  geometry_msgs::Vector3 thrust_rate;
  ros::Publisher thrust_rate_publisher_ =
      nh.advertise<geometry_msgs::Vector3>("/thrust_rate", 10);

  geometry_msgs::Vector3 torque_rate;
  ros::Publisher torque_rate_publisher_ =
      nh.advertise<geometry_msgs::Vector3>("/torque_rate", 10);

  // Thrust Message
  geometry_msgs::Vector3 thrust_command_ros;
  // Torque Message
  geometry_msgs::Vector3 torque_command_ros;
  // Thrust Publisher
  ros::Publisher thrust_command_publisher_ =
      nh.advertise<geometry_msgs::Vector3>("/command/thrust", 10);
  // Torque Publisher
  ros::Publisher torque_command_publisher_ =
      nh.advertise<geometry_msgs::Vector3>("/command/torque", 10);

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
      controller.publish_optimal_rollout();
      controller.publish_trajectories();
    } else {
      controller.set_observation(x, sim_time);
      controller.get_input_state(x, x_nom, u, sim_time);
    }
    if (!static_optimization) {
      x = simulation->step(u, sim_dt);
      simulation->publish_ros();
      sim_time += sim_dt;
    }

    // Publish  command and publish it
    omav_raisim::conversions::to_thrust(x_nom, thrust_command_ros);
    omav_raisim::conversions::to_torque(x_nom, torque_command_ros);

    thrust_command_publisher_.publish(thrust_command_ros);
    torque_command_publisher_.publish(torque_command_ros);

    // Publish Rates
    omav_raisim::conversions::to_thrust_rate(u, thrust_rate);
    omav_raisim::conversions::to_torque_rate(u, torque_rate);
    thrust_rate_publisher_.publish(thrust_rate);
    torque_rate_publisher_.publish(torque_rate);

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
