/*!
 * @file     panda_raisim.cpp
 * @author   Giuseppe Rizzi
 * @date     05.10.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_panda/dynamics_raisim.h"
#include <chrono>
#include <thread>
#include <numeric>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#define TIMESTEP 0.01

using namespace panda;
using namespace std::chrono;

int main(int argc, char** argv) {
  ros::init(argc, argv, "panda_raisim");
  ros::NodeHandle nh("~");

  auto robot_description_raisim = nh.param<std::string>("/robot_description_raisim", "");
  auto simulation = std::make_shared<PandaRaisimDynamics>(robot_description_raisim, TIMESTEP);

  Eigen::VectorXd u;
  u.setZero(PandaDim::INPUT_DIMENSION);
  u(0) = 0.1;

  Eigen::VectorXd x;
  x.setZero(PandaDim::STATE_DIMENSION);
  x << 0.0, -0.52, 0.0, -1.785, 0.0, 1.10, 0.69, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  std::cout << "Resetting to state x=" << x.transpose() << std::endl;
  simulation->reset(x);

  ros::Publisher state_publisher = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
  sensor_msgs::JointState joint_state;
  joint_state.name = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5",
                      "panda_joint6", "panda_joint7"};
  joint_state.position.resize(7);
  joint_state.header.frame_id = "world";

  std::vector<double> times;
  std::cout << "Starting simulation!" << std::endl;
  long int loopN = 2000;
  for (size_t i = 0; i < loopN; i++) {
    auto start = steady_clock::now();
    x = simulation->step(u, TIMESTEP);
    auto end = steady_clock::now();
    times.push_back(duration_cast<nanoseconds>(end-start).count()/1e6);


    if (i == 1000){
      std::cout << "Resetting simulation with speed in the first joint." << std::endl;
      x << 0.0, -0.52, 0.0, -1.785, 0.0, 1.10, 0.69, -20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      simulation->reset(x);
    }

    joint_state.header.stamp = ros::Time::now();
    for (size_t j = 0; j < 7; j++) {
      joint_state.position[j] = x(j);
    }
    state_publisher.publish(joint_state);

    ros::Duration(TIMESTEP).sleep();
    ROS_INFO_STREAM_THROTTLE(1.0, "Sim time: " << i*TIMESTEP << " s.");
  }

  std::cout << "Average sim time: " << std::accumulate(times.begin(), times.end(), 0.0)/times.size();
  return 0;
}
