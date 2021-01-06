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
#include "mppi_manipulation/dynamics.h"

#define TIMESTEP 0.01
#define DEFAULT_CONFIGURATION 0.0, -0.52, 0.0, -1.785, 0.0, 1.10, 0.69, 0.04, 0.04;
#define OBJECT_CONFIGURATION 0.0, 0.0

using namespace manipulation;
using namespace std::chrono;

int main(int argc, char** argv) {
  raisim::World::setActivationKey("/home/giuseppe/git/raisimlib/rsc/activation.raisim");

  ros::init(argc, argv, "panda_raisim");
  ros::NodeHandle nh("~");

  // Instantiate the simulation world
  auto robot_description_raisim = nh.param<std::string>("/robot_description_raisim", "");
  auto object_description_raisim = nh.param<std::string>("/object_description_raisim", "");
  PandaRaisimDynamics simulation(robot_description_raisim, object_description_raisim, TIMESTEP);

  // Reset the state to a default configuration
  Eigen::VectorXd x;
  Eigen::VectorXd x_snapshot;
  x.setZero(PandaDim::STATE_DIMENSION);
  x_snapshot.setZero(PandaDim::STATE_DIMENSION);
  x.head<PandaDim::JOINT_DIMENSION>() << DEFAULT_CONFIGURATION;
  x.segment<2 * PandaDim::OBJECT_DIMENSION>(2 * PandaDim::JOINT_DIMENSION) << OBJECT_CONFIGURATION;

  std::cout << "Resetting to state x=" << x.transpose() << std::endl;
  simulation.reset(x);

  // Instantiate the input vector to hit the object and slowly close gripper
  Eigen::VectorXd u;
  Eigen::VectorXd u_snapshot;
  u = simulation.get_zero_input(x);
  u_snapshot = simulation.get_zero_input(x);
  u(1) = 0.1;
  u(3) = 0.1;
  u.tail<1>()(0) = -0.005;
  std::cout << "Initializing input to: " << u.transpose() << std::endl;

  // Ros publishing for visualization
  ros::Publisher state_publisher = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
  ros::Publisher object_state_publisher =
      nh.advertise<sensor_msgs::JointState>("/object/joint_state", 10);
  sensor_msgs::JointState joint_state, object_state;
  joint_state.name = {"panda_joint1", "panda_joint2",        "panda_joint3",
                      "panda_joint4", "panda_joint5",        "panda_joint6",
                      "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"};
  joint_state.position.resize(joint_state.name.size());
  joint_state.velocity.resize(joint_state.name.size());
  joint_state.header.frame_id = "world";
  object_state.name = {"articulation_joint"};
  object_state.position.resize(1);

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

    joint_state.header.stamp = ros::Time::now();
    for (size_t j = 0; j < PandaDim::JOINT_DIMENSION; j++) {
      joint_state.position[j] = x(j);
      joint_state.velocity[j] = x(j + PandaDim::JOINT_DIMENSION);
    }
    state_publisher.publish(joint_state);

    object_state.header.stamp = ros::Time::now();
    object_state.position[0] = x(PandaDim::JOINT_DIMENSION * 2);
    object_state_publisher.publish(object_state);

    raisim::MSLEEP(10);
    ROS_INFO_STREAM_THROTTLE(
        1.0, "Sim time: " << i * TIMESTEP << " s, elapsed sim dt: " << elapsed << " ms.");
  }

  std::cout << "Average sim time: " << total_time / loopN << std::endl;
  return 0;
}
