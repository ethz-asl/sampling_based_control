#include "mppi_manipulation_royalpanda/controller_standalone.h"
#include <ros/ros.h>

using namespace manipulation_royalpanda;

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_observer");
  ros::NodeHandle nh("~"); 
  ros::NodeHandle controller_nh("manipulation_controller");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ManipulationController controller;
  if (!controller.init(nh, controller_nh)){
  	ROS_WARN("Failed to init the controller");
  }

  controller.starting(ros::Time::now());
  ros::Rate rate(1000);
  while (ros::ok()) {
    controller.update(ros::Time::now(), ros::Duration().fromSec(0.001));
    rate.sleep();
  }
  spinner.stop();
  return 0;
}
