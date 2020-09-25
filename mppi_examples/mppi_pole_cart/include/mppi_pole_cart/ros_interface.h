/*!
 * @file     mppi_ros_interface.h
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#pragma once
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

namespace pole_cart{

class PoleCartRosInterface{
 public:
  explicit PoleCartRosInterface(ros::NodeHandle& nodeHandle): nh(nodeHandle) {
    state_publisher = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    ghost_state_publisher = nh.advertise<sensor_msgs::JointState>("/joint_ghost_states", 10);

    force_publisher = nh.advertise<std_msgs::Float64>("/force", 10);
    cost_publisher = nh.advertise<std_msgs::Float64>("/cost", 10);

    joint_state.name.push_back("cart");
    joint_state.name.push_back("pendulum");
    joint_state.position.resize(2);
    joint_state.header.frame_id = "odom";

    ghost_joint_state.name.push_back("cart_ghost");
    ghost_joint_state.name.push_back("pendulum_ghost");
    ghost_joint_state.position.resize(2);
    ghost_joint_state.header.frame_id = "odom";

  };
  ~PoleCartRosInterface() = default;
 public:
  void publish_state(double x, double theta){
    joint_state.header.stamp = ros::Time::now();
    joint_state.position[0] = x;
    joint_state.position[1] = theta;
    this->state_publisher.publish(joint_state);
  }

  void publish_ghost_state(double x, double theta){
    ghost_joint_state.header.stamp = ros::Time::now();
    ghost_joint_state.position[0] = x;
    ghost_joint_state.position[1] = theta;
    this->ghost_state_publisher.publish(ghost_joint_state);
  }

  void publish_force(double F){
    force.data = F;
    this->force_publisher.publish(force);
  }

  void publish_cost(double c){
    cost.data = c;
    this->cost_publisher.publish(cost);
  }

 private:
  ros::NodeHandle nh;
  ros::Publisher state_publisher;
  ros::Publisher ghost_state_publisher;

  ros::Publisher force_publisher;
  ros::Publisher cost_publisher;

  sensor_msgs::JointState joint_state;
  sensor_msgs::JointState ghost_joint_state;

  std_msgs::Float64 force;
  std_msgs::Float64 cost;

};
}
