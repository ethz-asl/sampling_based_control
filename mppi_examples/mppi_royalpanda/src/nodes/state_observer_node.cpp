//
// Created by giuseppe on 29.01.21.
//

#include <ros/ros.h>
#include "mppi_royalpanda/state_observer.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_observer_test");
  ros::NodeHandle nh("~");

  double frequency;
  nh.param<double>("observer_update_frequency", frequency, 200);

  royalpanda::StateObserver observer(nh);
  if (!observer.initialize()) {
    ROS_ERROR("Failed to initialize the state observer.");
    return 0;
  }

  ros::Rate rate(frequency);
  while (ros::ok()) {
    observer.update();
    observer.publish();

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
