//
// Created by giuseppe on 25.04.21.
//

#include "mppi_ros/ros_params.h"

namespace mppi_ros {
bool getString(ros::NodeHandle& nh, const std::string& param_name,
               std::string& obj) {
  if (!nh.getParam(param_name, obj)) {
    ROS_ERROR_STREAM("Failed to parse param " << param_name);
    return false;
  }

  if (obj.empty()) {
    ROS_ERROR_STREAM("Failed to parse param " << param_name
                                              << ". String is empty.");
    return false;
  }
  return true;
}

bool getBool(ros::NodeHandle& nh, const std::string& param_name, bool& obj) {
  if (!nh.getParam(param_name, obj)) {
    ROS_ERROR_STREAM("Failed to parse param " << param_name);
    return false;
  }
  return true;
}
}  // namespace mppi_ros
