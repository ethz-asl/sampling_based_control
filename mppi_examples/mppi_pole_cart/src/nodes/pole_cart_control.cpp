/*!
 * @file     pole_cart_control.cpp
 * @author   Giuseppe Rizzi
 * @date     11.06.2020
 * @version  1.0
 * @brief    description
 */
#include "mppi_pole_cart/controller_interface.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <chrono>

using namespace pole_cart;

int main(int argc, char** argv) {
  // ros interface
  ros::init(argc, argv, "panda_control_node");
  ros::NodeHandle nh("~");
  auto controller = PoleCartControllerInterface(nh);

  auto dynamics_config = PoleCartDynamicsConfig();
  dynamics_config.mc = nh.param<double>("dynamics/mass_cart", 1.0);
  dynamics_config.mp = nh.param<double>("dynamics/mass_pendulum", 0.5);
  dynamics_config.l = nh.param<double>("dynamics/length", 1.0);
  dynamics_config.mux = nh.param<double>("dynamics/linear_friction", 10.0);
  dynamics_config.mutheta = nh.param<double>("dynamics/angular_friction", 0.7);
  dynamics_config.dt_internal =
      nh.param<double>("dynamics/substep_size", 0.001);
  PoleCartDynamics simulation(dynamics_config);

  Eigen::VectorXd x = Eigen::VectorXd::Zero(PoleCartDim::STATE_DIMENSION);
  x(0) = 0.0;
  x(1) = 0.0;
  x(2) = 0.1;
  simulation.reset(x);

  mppi::DynamicsBase::input_t u;
  u = simulation.get_zero_input(x);
  std::cout << "First input: " << u.transpose() << std::endl;

  ros::Publisher state_publisher =
      nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
  sensor_msgs::JointState joint_state;
  joint_state.name.push_back("cart");
  joint_state.name.push_back("pendulum");
  joint_state.position.resize(2);
  joint_state.header.frame_id = "world";

  bool static_optimization = nh.param<bool>("static_optimization", false);
  double sim_dt = nh.param<double>("sim_dt", 0.01);

  double sim_time = 0.0;

  // init the controller
  bool ok = controller.init();
  if (!ok) {
    throw std::runtime_error("Failed to initialzied controller!");
  }

  // set the very first observation
  controller.set_observation(x, sim_time);

  // sim loop
  controller.start();
  while (ros::ok()) {
    auto start = std::chrono::steady_clock::now();
    controller.set_observation(x, sim_time);
    controller.get_input(x, u, sim_time);
    if (!static_optimization) {
      x = simulation.step(u, sim_dt);
      sim_time += sim_dt;
    }

    for (size_t i = 0; i < 2; i++) joint_state.position[i] = x(i);
    joint_state.header.stamp = ros::Time::now();
    state_publisher.publish(joint_state);

    auto end = std::chrono::steady_clock::now();
    double elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count() *
        1000;
    if (sim_dt - elapsed > 0) ros::Duration(sim_dt - elapsed).sleep();

    ros::spinOnce();
  }
}