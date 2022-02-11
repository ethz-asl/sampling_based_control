//
// Created by studigem on 13.06.21.
//
#include "mppi_omav_interaction/state_observer.h"
#include "mppi_omav_interaction/state_observer_valve.h"

template <class T>
void getParam(const ros::NodeHandle &nh, const std::string &id, T &var,
              const T &val_def) {
  if (!nh.param<T>(id, var, val_def)) {
    ROS_FATAL_STREAM("[state_observer_node] Could not find parameter " << id);
    ros::shutdown();
    exit(1);
  } else {
    ROS_INFO_STREAM("[state_observer_node] Successfully read " << id);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "state_observer_object");
  ros::NodeHandle nh("~");
  std::string object_name;
  double observer_rate = 100.0;
  getParam<std::string>(nh, "object_name", object_name, "shelf");
  getParam<double>(nh, "observer_rate", observer_rate, observer_rate);

  if (object_name.compare("shelf") == 0 || object_name.compare("shelf_door") == 0) {
    object_observer::StateObserver observer(nh);
    if (!observer.initialize()) {
      ROS_ERROR(
          "[state_observer_node] Failed to initialize the state observer.");
      return 0;
    }
    ROS_INFO("[state_observer_node] Starting state observer for shelf.");
    ros::Rate rate(observer_rate);
    while (ros::ok()) {
      observer.publish();
      ros::spinOnce();
      rate.sleep();
    }
  } else if (object_name.compare("valve") == 0) {
    object_observer::StateObserverValve observer(nh);
    if (!observer.initialize()) {
      ROS_ERROR(
          "[state_observer_node] Failed to initialize the state observer.");
      return 0;
    }
    ROS_INFO("[state_observer_node] Starting state observer for valve.");
    ros::Rate rate(observer_rate);
    while (ros::ok()) {
      observer.publish();
      ros::spinOnce();
      rate.sleep();
    }
  } else {
    ROS_ERROR("[state_observer_node] Wrong object name: %s, shutting down.",
              object_name.c_str());
    ros::shutdown();
  }
  return 0;
}
