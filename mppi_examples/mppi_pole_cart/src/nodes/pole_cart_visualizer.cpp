/*!
 * @file     pole_cart_visualizer.cpp
 * @author   Giuseppe Rizzi
 * @date     23.07.2020
 * @version  1.0
 * @brief    description
 */

#include <ros/ros.h>
#include <chrono>
#include <mppi/solver_config.h>
#include <mppi_ros/visualization/mppi_visualizer_ros.h>
#include <mppi/sampler/gaussian_sampler.h>

#include "mppi_pole_cart/dynamics.h"
#include "mppi_pole_cart/ros_interface.h"
#include "mppi_pole_cart/cost.h"
#include <std_msgs/Bool.h>

using namespace pole_cart;

class PoleCartPIVisualizer : public mppi_ros::PIVisualizerRos{
 public:
  PoleCartPIVisualizer(mppi::DynamicsBase::dynamics_ptr dynamics,
                       mppi::CostBase::cost_ptr cost,
                       mppi::SamplerBase::sampler_ptr sampler,
                       const mppi::SolverConfig& config,
                       ros::NodeHandle& nh):
  PIVisualizerRos(dynamics, cost, sampler, config, nh),
  ros_interface(nh){};

  void visualize_single_trajectory(const observation_array_t& traj, const double dt) override{
    for(const auto& x : traj){
      ros_interface.publish_ghost_state(x(0), x(1));
      if (dt>0) ros::Duration(dt).sleep();
    }
  }

  void visualize_optimal_trajectory(const observation_array_t& traj) override {
    visualize_single_trajectory(traj, config_.step_size);
  }

 private:
  PoleCartRosInterface ros_interface;
};

int main(int argc, char** argv){
  // ros interface
  ros::init(argc, argv, "pendulum_cart_visualizer_node");
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
  auto config_file = nh.param<std::string>("config_file", "");
  if (!solver_config.init_from_file(config_file)){
    ROS_ERROR_STREAM("Failed to init solver options from " << config_file);
    return -1;
  }

  auto solver_visualizer = PoleCartPIVisualizer(dynamics, cost, sampler, solver_config, nh);

  double sim_time=0;
  double f;
  mppi::DynamicsBase::input_t u;
  double stage_cost;

  ros::Duration(5.0).sleep();
  size_t step_count = 0;
  mppi::state_input_pair xu;
  xu.x = x;
  xu.u = u;
  while (ros::ok()){

    xu = solver_visualizer.run(xu.x, sim_time, step_count);
    step_count++;
    sim_time = step_count * solver_config.step_size;

    // publish to ros
    solver_visualizer.publish_ros();
    stage_cost = cost->get_stage_cost(xu.x);
    ros_interface.publish_state(xu.x(0), xu.x(1));
    ros_interface.publish_force(xu.u(0));
    ros_interface.publish_cost(stage_cost);
    ros::spinOnce();
  }
}