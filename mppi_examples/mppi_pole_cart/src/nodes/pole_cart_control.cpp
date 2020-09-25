/*!
 * @file     pendulum_cart_control.cpp
 * @author   Giuseppe Rizzi
 * @date     11.06.2020
 * @version  1.0
 * @brief    description
 */

#include <ros/ros.h>
#include <chrono>
#include <mppi/solver_config.h>
#include <mppi/controller/mppi.h>
#include <mppi/sampler/gaussian_sampler.h>

#include "mppi_pole_cart/dynamics.h"
#include "mppi_pole_cart/ros_interface.h"
#include "mppi_pole_cart/cost.h"

using namespace pole_cart;
using namespace std::chrono;

int main(int argc, char** argv){

  // ros interface
  ros::init(argc, argv, "pendulum_cart_control_node");
  ros::NodeHandle nh("~");
  auto ros_interface = PoleCartRosInterface(nh);

  // dynamics and cost
  auto dynamics_config = PoleCartDynamicsConfig();
  dynamics_config.mc = nh.param<double>("dynamics/mass_cart", 1.0);
  dynamics_config.mp = nh.param<double>("dynamics/mass_pendulum", 0.5);
  dynamics_config.l = nh.param<double>("dynamics/length", 1.0);
  dynamics_config.mux = nh.param<double>("dynamics/linear_friction", 10.0);
  dynamics_config.mutheta = nh.param<double>("dynamics/angular_friction", 0.7);
  dynamics_config.dt_internal = nh.param<double>("dynamics/substep_size", 0.001);

  auto pole_cart_dynamics = PoleCartDynamics();
  pole_cart_dynamics.set_dynamic_properties(dynamics_config);
  mppi::DynamicsBase::dynamics_ptr dynamics = pole_cart_dynamics.clone();
  mppi::DynamicsBase::dynamics_ptr simulation = pole_cart_dynamics.clone();

  mppi::CostBase::cost_ptr cost = std::make_shared<PoleCartCost>();

  // set initial state
  PoleCartDynamics::input_t x = PoleCartDynamics::input_t::Zero(PoleCartDim::STATE_DIMENSION);
  x(1) = 0.0;
  x(3) = 0.0;
  pole_cart_dynamics.reset(x);

  // sampler
  auto noise_sampler = mppi::GaussianSampler(PoleCartDim::INPUT_DIMENSION);
  double noise_sigma = nh.param<double>("noise_variance", 1.0);
  noise_sampler.set_noise_variance(noise_sigma);
  mppi::SamplerBase::sampler_ptr sampler = std::shared_ptr<mppi::GaussianSampler>(&noise_sampler);

  // solver config
  auto solver_config = mppi::SolverConfig();
  std::string config_file = nh.param<std::string>("config_file", "");
  if (!solver_config.init_from_file(config_file)){
    ROS_ERROR_STREAM("Failed to init solver options from " << config_file);
    return -1;
  }

  auto solver = mppi::PathIntegral(dynamics, cost, sampler, solver_config);

  double sim_time=0.0;
  mppi::DynamicsBase::input_t u = Eigen::VectorXd::Zero(PoleCartDim::INPUT_DIMENSION);
  double stage_cost;

  ros::Duration(5.0).sleep();
  size_t iterations = 0;
  std::mutex x_mutex;
  std::mutex u_mutex;

  // thread 1: control loop
  std::thread tc([&](){
    while(ros::ok()){
      auto start = steady_clock::now();
      solver.update_policy();
      auto end = steady_clock::now();
      ROS_INFO_STREAM_THROTTLE(1.0, "Controller update took: " << duration_cast<milliseconds>(end - start).count() << " ms.");
    }
  });


  // thread 2: sim loop
  std::thread ts([&](){
    while(ros::ok()){
      {
        std::lock_guard<std::mutex> xlock(x_mutex);
        std::lock_guard<std::mutex> ulock(u_mutex);
        solver.set_observation(x, sim_time);
        solver.get_input(x, u, sim_time);
        x = simulation->step(u, solver_config.step_size);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds((int)(solver_config.step_size * 1000.)));
      sim_time += solver_config.step_size;
    }
  });


  std::thread tr([&](){
    // thread 3: publish to ros
    while(ros::ok()) {
      std::lock_guard<std::mutex> xlock(x_mutex);
      std::lock_guard<std::mutex> ulock(u_mutex);
      ros_interface.publish_state(x(0), x(1));
      ros_interface.publish_force(u(0));
      ros_interface.publish_cost(cost->get_stage_cost(x));
    }
  });

  tc.join();
  ts.join();
  tr.join();

//   try sequentially first
//  while(ros::ok()){
//    x = simulation->step(u, solver_config.step_size);
//    solver.set_observation(x, sim_time);
//    solver.get_input(x, u, sim_time);
//    solver.update_policy();
//
//    ros_interface.publish_state(x(0), x(1));
//    ros_interface.publish_force(u(0));
//    ros_interface.publish_cost(cost->get_stage_cost(x));
//    sim_time += solver_config.step_size;
//    //std::this_thread::sleep_for(std::chrono::milliseconds((int)(solver_config.step_size * 1000.)));
//  }
}