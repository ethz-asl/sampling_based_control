/*!
 * @file     pendulum_cart_control.cpp
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#include <ros/ros.h>
#include "mppi_pole_cart/cost.h"
#include "mppi_pole_cart/dynamics.h"
#include "mppi_pole_cart/ros_interface.h"

using namespace pole_cart;
int main(int argc, char** argv){
  ros::init(argc, argv, "pendulum_cart_control_node");
  ros::NodeHandle nh("~");
  auto ros_interface = PoleCartRosInterface(nh);

  PoleCartDynamicsConfig config;
  config.mc = nh.param<double>("dynamics/mass_cart", 1.0);
  config.mp = nh.param<double>("dynamics/mass_pendulum", 0.5);
  config.l = nh.param<double>("dynamics/length", 1.0);
  config.mux = nh.param<double>("dynamics/linear_friction", 10.0);
  config.mutheta = nh.param<double>("dynamics/angular_friction", 0.7);
  config.dt_internal = nh.param<double>("dynamics/substep_size", 0.001);

  auto pole_cart_dynamics = PoleCartDynamics();
  pole_cart_dynamics.set_dynamic_properties(config);
  PoleCartDynamics::observation_t x = PoleCartDynamics::input_t::Zero(PoleCartDim::STATE_DIMENSION);
  x(1) = M_PI;
  x(3) = 0.001;
  pole_cart_dynamics.reset(x);

  double dt = 0.01;
  double cost;
  ros::Rate rate(1.0/dt);
  auto pendulum_cart_cost = PoleCartCost();
  auto u = PoleCartDynamics::input_t::Zero(PoleCartDim::INPUT_DIMENSION);
  while (ros::ok()){
    x = pole_cart_dynamics.step(u, dt);
    cost = pendulum_cart_cost.get_stage_cost(x);
    ros_interface.publish_state(x(0), x(1));
    ros_interface.publish_force(0.0);
    ros_interface.publish_cost(cost);
    rate.sleep();
  }
}
