//
// Created by studigem on 30.03.21.
//

#include "mppi_omav_velocity/ros_conversions.h"

namespace omav_velocity::conversions {
void to_trajectory_msg(
    const mppi::observation_array_t &x_opt,
    trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg) {
  double dt = 0.015;
  mav_msgs::EigenTrajectoryPoint current_trajectory_point;
  mav_msgs::EigenTrajectoryPointVector current_trajectory;
  for (int i = 1; i < 30; i++) {
    EigenTrajectoryPointFromState(x_opt, i, current_trajectory_point, dt);
    current_trajectory.push_back(current_trajectory_point);
  }
  mav_msgs::msgMultiDofJointTrajectoryFromEigen(current_trajectory,
                                                &trajectory_msg);
}

void EigenTrajectoryPointFromState(
    const observation_array_t &states, int i,
    mav_msgs::EigenTrajectoryPoint &trajectorypoint, double dt) {
  trajectorypoint.position_W = states[i].head<3>();
  trajectorypoint.orientation_W_B =
      Eigen::Quaternion(states[i](3), states[i](4), states[i](5), states[i](6));
  trajectorypoint.velocity_W = states[i].segment<3>(7);
  trajectorypoint.angular_velocity_W = states[i].segment<3>(10);
  trajectorypoint.acceleration_W =
      (states[i].segment<3>(7) - states[i - 1].segment<3>(7)) / dt;
  trajectorypoint.angular_acceleration_W =
      (states[i].segment<3>(10) - states[i - 1].segment<3>(10)) / dt;
  trajectorypoint.time_from_start_ns = ros::Duration(i * dt).toNSec();
}
void PoseMsgFromVector(const Eigen::VectorXd &pose,
                       geometry_msgs::PoseStamped &pose_msg) {
  pose_msg.pose.position.x = pose(0);
  pose_msg.pose.position.y = pose(1);
  pose_msg.pose.position.z = pose(2);

  pose_msg.pose.orientation.w = pose(3);
  pose_msg.pose.orientation.x = pose(4);
  pose_msg.pose.orientation.y = pose(5);
  pose_msg.pose.orientation.z = pose(6);
}
} // namespace omav_velocity::conversions
