//
// Created by giuseppe on 09.02.21.
//

#include <manipulation_msgs/InputState.h>
#include <manipulation_msgs/conversions.h>
#include <ros/ros.h>
#include "mppi_manipulation/controller_interface.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "manipulation_controller");
  ros::NodeHandle nh("~");

  bool observation_set = false;
  Eigen::VectorXd x_;
  Eigen::VectorXd x_nom_;
  Eigen::VectorXd u_;

  manipulation::PandaControllerInterface man_interface(nh);
  if (!man_interface.init()) {
    ROS_ERROR("Failed to initialize controller manipulation interface.");
    return 0;
  }

  auto cb = [&](const manipulation_msgs::StateConstPtr& msg) {
    manipulation::conversions::msgToEigen(*msg, x_);
    man_interface.set_observation(x_, msg->header.stamp.toSec());
    if (!observation_set) {
      man_interface.start();
      ROS_INFO(
          "[Controller Node]: first observation received and controller "
          "started!");
    }
    observation_set = true;
  };

  ros::Subscriber state_subscriber =
      nh.subscribe<manipulation_msgs::State>("/observer/state", 1, cb);

  manipulation_msgs::InputState input_state_msg;
  ros::Publisher input_publisher =
      nh.advertise<manipulation_msgs::InputState>("/controller/input_state", 1);

  ros::Rate rate(100);
  while (ros::ok()) {
    if (observation_set) {
      man_interface.get_input_state(x_, x_nom_, u_, ros::Time::now().toSec());
      manipulation::conversions::eigenToMsg(u_, input_state_msg.input);
      manipulation::conversions::eigenToMsg(x_nom_, input_state_msg.state);
      input_publisher.publish(input_state_msg);
    } else {
      ROS_WARN_STREAM_THROTTLE(
          2.0, "[Controller Node]: waiting to receive the first observation.");
    }
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}