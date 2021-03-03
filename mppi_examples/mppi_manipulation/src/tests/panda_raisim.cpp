/*!
 * @file     panda_raisim.cpp
 * @author   Giuseppe Rizzi
 * @date     05.10.2020
 * @version  1.0
 * @brief    Simulation of the panda dynamics class with raisim backend
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <chrono>
#include <numeric>
#include <thread>
#include "mppi_manipulation/dynamics_ros.h"

#define TIMESTEP 0.01

using namespace manipulation;
using namespace std::chrono;

int main(int argc, char** argv) {
  ros::init(argc, argv, "manipulation_sim");
  ros::NodeHandle nh("~");

  // activate raisim simulator
  std::string activation_file;
  nh.param<std::string>("activation_file", activation_file,
                        "/home/giuseppe/git/raisimlib/rsc/activation.raisim");
  raisim::World::setActivationKey(activation_file);

  // Fixed base option
  bool fixed_base;
  if (!nh.param<bool>("fixed_base", fixed_base, false)) {
    ROS_ERROR_STREAM("Failed to find param fixed_base");
    return -1;
  }

  // Instantiate the simulation world
  auto robot_description_raisim =
      nh.param<std::string>("/robot_description_raisim", "");
  auto object_description_raisim =
      nh.param<std::string>("/object_description_raisim", "");
  ManipulatorDynamicsRos simulation(nh, robot_description_raisim,
                                    object_description_raisim, TIMESTEP,
                                    fixed_base);

  // Reset the state to a default configuration
  Eigen::VectorXd x, x_snapshot;
  simulation.reset_to_default();

  // Instantiate the input vector to hit the object and slowly close gripper
  Eigen::VectorXd u = Eigen::VectorXd::Zero(simulation.get_input_dimension());
  Eigen::VectorXd u_snapshot =
      Eigen::VectorXd::Zero(simulation.get_input_dimension());
  u(1) = 0.2;
  u(2) = 0.1;
  u(4) = 0.1;
  u(6) = 0.1;
  std::cout << "Initializing input to: " << u.transpose() << std::endl;

  long int loopN = 2000;
  double elapsed = 0;
  double total_time = 0;
  std::chrono::time_point<steady_clock> start, end;

  std::cout << "Starting simulation!" << std::endl;
  for (size_t i = 0; i < loopN; i++) {
    start = steady_clock::now();
    x = simulation.step(u, TIMESTEP);
    end = steady_clock::now();
    elapsed = duration_cast<nanoseconds>(end - start).count() / 1e6;
    total_time += elapsed;

    if (i == 500) {
      std::cout << "Taking snapshot" << std::endl;
      x_snapshot = x;
      u_snapshot = u;
      std::cout << "x_snapshot: " << x_snapshot.transpose() << std::endl;
      std::cout << "u_snapshot: " << u_snapshot.transpose() << std::endl;
    }

    if (i == 1000) {
      std::cout << "Resetting simulation from snapshot" << std::endl;
      simulation.reset(x_snapshot);
      u = u_snapshot;
    }

    simulation.publish_ros();
    ROS_INFO_STREAM_THROTTLE(1.0, "Object displacement is: "
                                      << simulation.get_object_displacement());

    raisim::MSLEEP(10);
    ROS_INFO_STREAM_THROTTLE(
        1.0, "Sim time: " << i * TIMESTEP << " s, elapsed sim dt: " << elapsed
                          << " ms.");
  }

  std::cout << "Average sim time: " << total_time / loopN << std::endl;
  return 0;
}
