/*!
 * @file     panda_raisim.cpp
 * @author   Giuseppe Rizzi
 * @date     05.10.2020
 * @version  1.0
 * @brief    Simulation of the panda dynamics class with raisim backend
 */

#include "mppi_panda_raisim/dynamics.h"
#include <chrono>
#include <thread>
#include <numeric>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#define TIMESTEP 0.01
#define DEFAULT_CONFIGURATION 0.0, -0.52, 0.0, -1.785, 0.0, 1.10, 0.69;
using namespace panda;
using namespace std::chrono;


int main(int argc, char** argv) {
  raisim::World::setActivationKey("/home/giuseppe/git/raisimlib/rsc/activation.raisim");

  ros::init(argc, argv, "panda_raisim");
  ros::NodeHandle nh("~");

  // Instantiate the simulation world
  auto robot_description_raisim = nh.param<std::string>("/robot_description_raisim", "");
  PandaRaisimDynamics simulation(robot_description_raisim, TIMESTEP);

  // Reset the state to a default configuration
  Eigen::VectorXd x;
  Eigen::VectorXd x_snapshot;
  x.setZero(PandaDim::STATE_DIMENSION);
  x_snapshot.setZero(PandaDim::STATE_DIMENSION);
  x.head<7>() << DEFAULT_CONFIGURATION;
  x.tail<7>() << DEFAULT_CONFIGURATION;

  std::cout << "Resetting to state x=" << x.transpose() << std::endl;
  simulation.reset(x);

  // Instantiate the input vector with a small velocity in the first joint
  Eigen::VectorXd u;
  Eigen::VectorXd u_snapshot;
  u = simulation.get_zero_input(x);
  u_snapshot = simulation.get_zero_input(x);
  u(0) = 0.1;
  std::cout << "Initializing input to: " << u.transpose() << std::endl;

  // Ros publishing for visualization
  ros::Publisher state_publisher = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
  sensor_msgs::JointState joint_state;
  joint_state.name = {"panda_joint1",
                      "panda_joint2",
                      "panda_joint3",
                      "panda_joint4",
                      "panda_joint5",
                      "panda_joint6",
                      "panda_joint7"};
  joint_state.position.resize(7);
  joint_state.header.frame_id = "world";

  long int loopN = 2000;
  double elapsed=0;
  double total_time=0;
  std::chrono::time_point<steady_clock> start, end;

  std::cout << "Starting simulation!" << std::endl;
  for (size_t i = 0; i < loopN; i++) {
    start = steady_clock::now();
    x = simulation.step(u, TIMESTEP);
    end = steady_clock::now();
    elapsed = duration_cast<nanoseconds>(end-start).count()/1e6;
    total_time+=elapsed;
    std::cout << elapsed << std::endl;

    if (i == 500) {
      std::cout << "Taking snapshot" << std::endl;
      x_snapshot = x;
      u_snapshot = u;
    }

    if (i == 1000) {
      std::cout << "Resetting simulation from snapshot" << std::endl;
      simulation.reset(x_snapshot);
      u = u_snapshot;
    }

    joint_state.header.stamp = ros::Time::now();
    for (size_t j = 0; j < 7; j++) {
      joint_state.position[j] = x(j);
    }
    state_publisher.publish(joint_state);

    raisim::MSLEEP(10);
    ROS_INFO_STREAM_THROTTLE(1.0, "Sim time: " << i*TIMESTEP << " s, elapsed sim dt: " << elapsed << " ms.");
  }

  std::cout << "Average sim time: " << total_time/loopN << std::endl;
  return 0;
}
