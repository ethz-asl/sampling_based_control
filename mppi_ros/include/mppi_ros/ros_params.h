//
// Created by giuseppe on 01.03.21.
//

#pragma once
#include <ros/ros.h>

namespace mppi_ros {

template <typename T>
bool getNonNegative(ros::NodeHandle& nh, const std::string& param_name,
                    T& obj) {
  if (!nh.getParam(param_name, obj)) {
    ROS_ERROR_STREAM("Failed to parse param " << param_name);
    return false;
  }

  if (obj < 0) {
    ROS_ERROR_STREAM("Failed to parse param " << param_name
                                              << ". Invalid value: " << obj);
    return false;
  }
  return true;
}

template <typename T>
bool getNVector(ros::NodeHandle& nh, const std::string& param_name,
                std::vector<T>& obj, size_t dim) {
  if (!nh.getParam(param_name, obj)) {
    ROS_ERROR_STREAM("Failed to parse param " << param_name);
    return false;
  }

  if (obj.size() != dim) {
    ROS_ERROR_STREAM("Failed to parse param "
                     << param_name << ". Invalid vector size: " << obj.size());
    return false;
  }
  return true;
}

bool getString(ros::NodeHandle& nh, const std::string& param_name,
               std::string& obj);

bool getBool(ros::NodeHandle& nh, const std::string& param_name, bool& obj);
}  // namespace mppi_ros