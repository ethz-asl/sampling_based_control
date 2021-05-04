//
// Created by giuseppe on 25.04.21.
//

#include <ros/ros.h>
#include "mppi_manipulation/model_tracking.h"

using namespace manipulation;

int main(int argc, char** argv) {
  // ros interface
  ros::init(argc, argv, "manipulation_model_tracking_node");
  ros::NodeHandle nh("~");
  ros::Rate rate(1 / .015);

  ManipulationTrackingController mt(nh);
  while (ros::ok()) {
    mt.step();
    mt.publish_ros();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}