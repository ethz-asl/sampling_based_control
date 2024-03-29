//
// Created by giuseppe on 29.01.21.
//

#include <ros/ros.h>
#include "mppi_manipulation_royalpanda/state_observer.h"

using namespace manipulation_royalpanda;

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_observer");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  StateObserver observer(nh);
  if (!observer.initialize()) {
    ROS_ERROR("Failed to initialize the state observer.");
    return 0;
  }

  ros::Rate rate(100);
  while (ros::ok()) {
    observer.publish_state();
    rate.sleep();
  }
  spinner.stop();
  return 0;
}
