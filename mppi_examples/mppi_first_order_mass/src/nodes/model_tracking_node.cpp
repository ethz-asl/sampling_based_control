//
// Created by giuseppe on 29.06.21.
//

#include "mppi_first_order_mass/model_tracking.h"

#include <ros/ros.h>

using namespace fom;

int main(int argc, char** argv) {
  // ros interface
  ros::init(argc, argv, "fom_model_tracking_node");
  ros::NodeHandle nh("~");

  FOMModelTracking mt(nh);
  while (ros::ok()) {
    mt.step();
    mt.publish_ros();
    ros::spinOnce();
  }

  return 0;
}