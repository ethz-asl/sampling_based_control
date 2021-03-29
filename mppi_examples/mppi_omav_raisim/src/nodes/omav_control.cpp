/*!
 * @file    omav_control.cpp
 * @author  Matthias Studiger
 * @date    18.03.2021
 * @version 1.0
 * @brief   description
 */
#include "mppi_omav_raisim/controller_interface.h"

#include <chrono>
#include <mav_msgs/TorqueThrust.h>
#include <memory>
#include <mppi_omav_raisim/dynamics_ros.h>
#include <ros/ros.h>
#include <vector>

using namespace omav_raisim;

int main(int argc, char **argv) {
  // Initialize Ros Node
  ros::init(argc, argv, "omav_raisim_control_node");
  ros::NodeHandle nh("~");

  // ros interface
  auto controller = OMAVControllerInterface(nh);

  auto simulation = std::make_shared<OMAVRaisimDynamicsRos>(nh, 0.015);
  // Implement all the publishers
  geometry_msgs::Vector3 ThrustRate;
  ros::Publisher ThrustRate_publisher_ =
      nh.advertise<geometry_msgs::Vector3>("/ThrustRate", 10);

  geometry_msgs::Vector3 TorqueRate;
  ros::Publisher TorqueRate_publisher_ =
      nh.advertise<geometry_msgs::Vector3>("/TorqueRate", 10);

  // Thrust Message
  geometry_msgs::Vector3 thrust_command_ros;
  // Torque Message
  geometry_msgs::Vector3 torque_command_ros;
  // Wrench Publisher
  mav_msgs::TorqueThrust TorqueThrust_command_ros;
  ros::Publisher TorqueThrust_command_publisher_ =
      nh.advertise<mav_msgs::TorqueThrust>("/command/TorqueThrust", 10);

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

  while (ros::ok()) {
    start = std::chrono::steady_clock::now();
    if (sequential) {
      controller.update_reference();
      controller.set_observation(x, sim_time);
      controller.update_policy();
      controller.get_input_state(x, x_nom, u, sim_time);
      std::cout << "---------------- x given to get_input state "
                   "---------------------------"
                << std::endl;
      std::cout << x.transpose() << std::endl;
      std::cout << "---------------- x_nom from get_input state "
                   "--------------------------"
                << std::endl;
      std::cout << x_nom.transpose() << std::endl;
      std::cout << "------------------ input u ----------------------"
                << std::endl;
      std::cout << u.transpose() << std::endl;
      std::cout << "------------------------------------------------------"
                << std::endl;
      controller.publish_ros_default();
      controller.publish_ros();
    } else {
      controller.set_observation(x, sim_time);
      controller.get_input_state(x, x_nom, u, sim_time);
    }

    if (!static_optimization) {
      x = simulation->step(u, sim_dt);
      std::cout << "-------------- x after simulation ------------------"
                << std::endl;
      std::cout << x.transpose() << std::endl;
      std::cout << "------------------------------------------------------"
                << std::endl;
      sim_time += sim_dt;
    }

    // Assemble wrench command and publish it
    thrust_command_ros.x = x_nom(0);
    thrust_command_ros.y = x_nom(1);
    thrust_command_ros.z = x_nom(2);
    torque_command_ros.x = x_nom(3);
    torque_command_ros.y = x_nom(4);
    torque_command_ros.z = x_nom(5);
    TorqueThrust_command_ros.thrust = thrust_command_ros;
    TorqueThrust_command_ros.torque = torque_command_ros;

    TorqueThrust_command_publisher_.publish(TorqueThrust_command_ros);

    // Publish Rates
    ThrustRate.x = u(0);
    ThrustRate.y = u(1);
    ThrustRate.z = u(2);
    ThrustRate_publisher_.publish(ThrustRate);

    TorqueRate.x = u(3);
    TorqueRate.y = u(4);
    TorqueRate.z = u(5);
    TorqueRate_publisher_.publish(TorqueRate);

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
