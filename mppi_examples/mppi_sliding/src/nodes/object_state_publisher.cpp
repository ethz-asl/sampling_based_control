//
// Created by giuseppe on 29.01.21.
//
#include <mppi_sliding/state_observer.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
using namespace manipulation_panda;

int main(int argc, char** argv) {
  ros::init(argc, argv, "object_publisher");
  ros::NodeHandle nh("~");

  ros::Publisher obj_publisher_;
  obj_publisher_ = nh.advertise<sensor_msgs::JointState>("/object/state", 10);
  sensor_msgs::JointState obj_msg;
  obj_msg.position.resize(7);
  obj_msg.velocity.resize(7);

  for (int i = 0; i < 7; i++) {
    obj_msg.position[i] = 0;
    obj_msg.velocity[i] = 0;
  }
  obj_msg.position[4] = 1;
  obj_msg.header.stamp = ros::Time::now();

  ros::Rate rate(100);
  while (ros::ok()) {
    obj_msg.header.stamp = ros::Time::now();
    rate.sleep();
    obj_publisher_.publish(obj_msg);
  }
  return 0;
}
