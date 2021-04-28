//
// Created by giuseppe on 01.03.21.
//

#include "mppi_pinocchio/ros_conversions.h"

namespace mppi_pinocchio {

void to_msg(const Pose& pose, geometry_msgs::msg::Pose& pose_ros) {
  pose_ros.position.x = pose.translation.x();
  pose_ros.position.y = pose.translation.y();
  pose_ros.position.z = pose.translation.z();
  pose_ros.orientation.x = pose.rotation.x();
  pose_ros.orientation.y = pose.rotation.y();
  pose_ros.orientation.z = pose.rotation.z();
  pose_ros.orientation.w = pose.rotation.w();
}
}  // namespace mppi_pinocchio