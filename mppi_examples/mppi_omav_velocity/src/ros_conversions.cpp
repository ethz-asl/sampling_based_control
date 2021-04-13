//
// Created by studigem on 30.03.21.
//

#include "mppi_omav_velocity/ros_conversions.h"

namespace omav_velocity::conversions {
void to_trajectory_msg(
    const mppi::observation_array_t &x_opt,
    trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg) {
  trajectory_msgs::MultiDOFJointTrajectoryPoint current_trajectory_point;
  geometry_msgs::Transform current_transform;
  geometry_msgs::Twist current_twist;
  trajectory_msg.header.frame_id = "odom";

  for (int i = 0; i < x_opt.size(); i++) {
    if ((i % 10) == 0) {
      current_transform.translation.x = x_opt[i](0);
      current_transform.translation.y = x_opt[i](1);
      current_transform.translation.z = x_opt[i](2);
      current_transform.rotation.w = x_opt[i](3);
      current_transform.rotation.x = x_opt[i](4);
      current_transform.rotation.y = x_opt[i](5);
      current_transform.rotation.z = x_opt[i](6);
      current_trajectory_point.transforms.push_back(current_transform);
      current_twist.linear.x = x_opt[i](7);
      current_twist.linear.y = x_opt[i](8);
      current_twist.linear.z = x_opt[i](9);
      current_twist.angular.x = x_opt[i](10);
      current_twist.angular.x = x_opt[i](11);
      current_twist.angular.x = x_opt[i](12);
      current_trajectory_point.velocities.push_back(current_twist);

      trajectory_msg.points.push_back(current_trajectory_point);
    }
  }
  trajectory_msg.header.stamp = ros::Time::now();
}
} // namespace omav_velocity::conversions
