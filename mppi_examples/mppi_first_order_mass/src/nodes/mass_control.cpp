/*!
 * @file     pole_cart_control.cpp
 * @author   Giuseppe Rizzi
 * @date     11.06.2020
 * @version  1.0
 * @brief    description
 */
#include <chrono>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include "mppi_first_order_mass/controller_interface.h"

using namespace fom;

int main(int argc, char** argv) {
  // ros interface
  ros::init(argc, argv, "mass_control_node");
  ros::NodeHandle nh("~");
  auto controller = FOMControllerInterface(nh);

  double sim_dt = nh.param<double>("sim_dt", 0.01);
  FOMDynamics simulation(sim_dt);

  Eigen::VectorXd x = Eigen::VectorXd::Zero(FOMDim::STATE_DIMENSION);
  x(0) = 0.0;
  x(1) = 0.0;
  simulation.reset(x, 0.0);

  mppi::input_t u;
  u = simulation.get_zero_input(x);

  ros::Publisher state_publisher =
      nh.advertise<geometry_msgs::Point>("/state", 10);
  geometry_msgs::Point state;

  bool static_optimization = nh.param<bool>("static_optimization", false);

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

    state.x = x[0];
    state.y = x[1];
    state_publisher.publish(state);

    auto end = std::chrono::steady_clock::now();
    double elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count() /
        1000;
    if (sim_dt - elapsed > 0) ros::Duration(sim_dt - elapsed).sleep();

    ros::spinOnce();
  }
}