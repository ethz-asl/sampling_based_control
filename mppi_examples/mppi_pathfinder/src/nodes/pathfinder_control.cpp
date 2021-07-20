/*!
 * @file     pathfinder_control.cpp
 * @author   Giuseppe Rizzi
 * @date     11.06.2020
 * @version  1.0
 * @brief    description
 */
#include "mppi_pathfinder/controller_interface.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <chrono>

using namespace pathfinder;

int main(int argc, char** argv) {
  // ros interface
  ros::init(argc, argv, "pathfinder_control_node");
  ros::NodeHandle nh("~");
  auto controller = PathfinderControllerInterface(nh);

  auto dynamics_config = PathfinderDynamicsConfig();
  dynamics_config.mc = nh.param<double>("dynamics/mass_cart", 1.0);
  dynamics_config.tau_theta = nh.param<double>("dynamics/tau_theta", 0.7);
  dynamics_config.dt_internal =
      nh.param<double>("dynamics/substep_size", 0.001);
  PathfinderDynamics simulation(dynamics_config);

  Eigen::VectorXd x = Eigen::VectorXd::Zero(PathfinderDim::STATE_DIMENSION);
  x(0) = 0.0;
  x(1) = 0.0;
  x(2) = 0.0;
  simulation.reset(x);

  mppi::DynamicsBase::input_t u;
  u = simulation.get_zero_input(x);
  std::cout << "First input: " << u.transpose() << std::endl;

  ros::Publisher state_publisher =
      nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
  sensor_msgs::JointState joint_state;
  joint_state.name.push_back("cart_x");
  joint_state.position.resize(1);
  joint_state.header.frame_id = "world";

  bool static_optimization = nh.param<bool>("static_optimization", false);
  double sim_dt = nh.param<double>("sim_dt", 0.01);

  double sim_time = 0.0;

  // init the controller
  bool ok = controller.init();
  if (!ok) {
    throw std::runtime_error("Failed to initialzied controller!");
  }

  std::cout << "First observation: " << x.transpose() << std::endl;
  // set the very first observation
  controller.set_observation(x, sim_time);

  // sim loop
  double max_vel = 0;
  double max_pos = 0;
  double max_pos_int = 0;

  controller.start();
  while (ros::ok()) {
    auto start = std::chrono::steady_clock::now();
    controller.set_observation(x, sim_time);
    controller.get_input(x, u, sim_time);
    controller.publish_optimal_rollout();
    if (!static_optimization) {
      x = simulation.step(u, sim_dt);
      sim_time += sim_dt;
    }
    max_vel = std::max(x(0), max_vel);
    max_pos = std::max(x(1), max_pos);
    max_pos_int = std::max(x(2), max_pos_int);
    std::cout << "----------" << std::endl;
    std::cout << max_vel << std::endl;
    std::cout << max_pos << std::endl;
    std::cout << max_pos_int << std::endl;

    joint_state.position[0] = x(2);
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