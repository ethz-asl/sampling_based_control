/*!
 * @file     pathfinder_control.cpp
 * @author   Giuseppe Rizzi
 * @date     11.06.2020
 * @version  1.0
 * @brief    description
 */
#include "mppi_OMAV_Pathfinder/controller_interface.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <chrono>

using namespace omav_pathfinder;

int main(int argc, char** argv) {
  // ros interface
  ros::init(argc, argv, "pathfinder_control_node");
  ros::NodeHandle nh("~");
  auto controller = OMAV_PathfinderControllerInterface(nh);

  auto dynamics_config = OMAV_PathfinderDynamicsConfig();
  dynamics_config.mass = nh.param<double>("dynamics/mass", 4.04);
  dynamics_config.gravity = nh.param<double>("dynamics/g", 9.8086);
  dynamics_config.Ix = nh.param<double>("dynamics/Ix", 0.078359);
  dynamics_config.Iy = nh.param<double>("dynamics/Iy", 0.081797);
  dynamics_config.Iz = nh.param<double>("dynamics/Iz", 0.153355);
  dynamics_config.dt_internal =
      nh.param<double>("dynamics/substep_size", 0.001);
  OMAV_PathfinderDynamics simulation(dynamics_config);

  Eigen::VectorXd x = Eigen::VectorXd::Zero(OMAV_PathfinderDim::STATE_DIMENSION);
  x(0) = 0.0;
  x(1) = 0.0;
  x(2) = 0.0;
  x(3) = 0.0;
  x(4) = 0.0;
  x(5) = 0.0;
  x(6) = 0.0;
  x(7) = 0.0;
  x(8) = 0.0;
  x(9) = 1.0;
  x(10) = 0.0;
  x(11) = 0.0;
  x(12) = 0.0;
  x(13) = 0.0;
  x(14) = 0.0;
  x(15) = 0.0;
  x(16) = 0.0;
  x(17) = 0.0;
  x(18) = 0.0;
  simulation.reset(x);

  mppi::DynamicsBase::input_t u;
  u = simulation.get_zero_input(x);
  std::cout << "First input: " << u.transpose() << std::endl;

  ros::Publisher state_publisher =
      nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
  sensor_msgs::JointState joint_state;
  joint_state.name.push_back("pos_x");
  joint_state.name.push_back("pos_y");
  joint_state.name.push_back("pos_z");
  joint_state.position.resize(3);
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

    for (size_t i = 0; i < 3; i++) joint_state.position[i] = x(i+16);
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