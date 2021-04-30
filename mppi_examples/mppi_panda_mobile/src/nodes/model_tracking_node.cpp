//
// Created by giuseppe on 25.04.21.
//

#include "mppi_panda_mobile/model_tracking.h"

#include <ros/ros.h>

using namespace panda_mobile;

int main(int argc, char** argv) {
  // ros interface
  ros::init(argc, argv, "panda_mobile_model_tracking_node");
  ros::NodeHandle nh("~");

  PandaMobileModelTracking mt(nh);
  while (ros::ok()) {
    mt.step();
    mt.publish_ros();
    ros::spinOnce();
  }

  return 0;
}