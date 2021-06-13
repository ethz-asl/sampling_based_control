//
// Created by studigem on 30.03.21.
//

#include "mppi_omav_interaction/ros_conversions.h"

namespace omav_interaction::conversions {
void to_trajectory_msg(
    const mppi::observation_array_t &x_opt,
    trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg) {
  double dt = 0.015;
  mav_msgs::EigenTrajectoryPoint current_trajectory_point;
  mav_msgs::EigenTrajectoryPointVector current_trajectory;

  for (int i = 0; i < (x_opt.size() - 6); i++) {
    EigenTrajectoryPointFromState(x_opt, i, current_trajectory_point, dt);
    current_trajectory.push_back(current_trajectory_point);
  }
  mav_msgs::msgMultiDofJointTrajectoryFromEigen(current_trajectory,
                                                &trajectory_msg);
}

void EigenTrajectoryPointFromState(
    const observation_array_t &states, int i,
    mav_msgs::EigenTrajectoryPoint &trajectorypoint, double dt) {

  // Window size is 2*m+1
  const size_t m = 3;
  // Polynomial Order
  const size_t n = 2;
  // Initial Point Smoothing (ie evaluate polynomial at first point in the
  // window)
  // Points are defined in range [-m;m]
  const size_t t = m;
  // Derivation order? 0: no derivation, 1: first derivative, 2: second
  // derivative...
  const int d = 1;
  gram_sg::SavitzkyGolayFilter filter(m, t, n, d);

  std::vector<double> lin_vel_x = {
      states[i](7),     states[i + 1](7), states[i + 2](7), states[i + 3](7),
      states[i + 4](7), states[i + 5](7), states[i + 6](7)};
  std::vector<double> lin_vel_y = {
      states[i](8),     states[i + 1](8), states[i + 2](8), states[i + 3](8),
      states[i + 4](8), states[i + 5](8), states[i + 6](8)};
  std::vector<double> lin_vel_z = {
      states[i](9),     states[i + 1](9), states[i + 2](9), states[i + 3](9),
      states[i + 4](9), states[i + 5](9), states[i + 6](9)};
  std::vector<double> ang_vel_x = {states[i](10),     states[i + 1](10),
                                   states[i + 2](10), states[i + 3](10),
                                   states[i + 4](10), states[i + 5](10),
                                   states[i + 6](10)};
  std::vector<double> ang_vel_y = {states[i](11),     states[i + 1](11),
                                   states[i + 2](11), states[i + 3](11),
                                   states[i + 4](11), states[i + 5](11),
                                   states[i + 6](11)};
  std::vector<double> ang_vel_z = {states[i](12),     states[i + 1](12),
                                   states[i + 2](12), states[i + 3](12),
                                   states[i + 4](12), states[i + 5](12),
                                   states[i + 6](12)};

  double lin_acc_x = filter.filter(lin_vel_x);
  double lin_acc_y = filter.filter(lin_vel_y);
  double lin_acc_z = filter.filter(lin_vel_z);
  double ang_acc_x = filter.filter(ang_vel_x);
  double ang_acc_y = filter.filter(ang_vel_y);
  double ang_acc_z = filter.filter(ang_vel_z);

  trajectorypoint.position_W = states[i].head<3>();
  trajectorypoint.orientation_W_B =
      Eigen::Quaternion(states[i](3), states[i](4), states[i](5), states[i](6));
  trajectorypoint.velocity_W = states[i].segment<3>(7);
  trajectorypoint.angular_velocity_W = states[i].segment<3>(10);
  trajectorypoint.acceleration_W.x() = lin_acc_x / dt;
  trajectorypoint.acceleration_W.y() = lin_acc_y / dt;
  trajectorypoint.acceleration_W.z() = lin_acc_z / dt;
  trajectorypoint.angular_acceleration_W.x() = ang_acc_x / dt;
  trajectorypoint.angular_acceleration_W.y() = ang_acc_y / dt;
  trajectorypoint.angular_acceleration_W.z() = ang_acc_z / dt;
  trajectorypoint.time_from_start_ns = ros::Duration(i * dt).toNSec();
}

void PoseStampedMsgFromVector(const Eigen::VectorXd &pose,
                              geometry_msgs::PoseStamped &pose_msg) {
  pose_msg.pose.position.x = pose(0);
  pose_msg.pose.position.y = pose(1);
  pose_msg.pose.position.z = pose(2);

  pose_msg.pose.orientation.w = pose(3);
  pose_msg.pose.orientation.x = pose(4);
  pose_msg.pose.orientation.y = pose(5);
  pose_msg.pose.orientation.z = pose(6);
}

void PoseMsgFromVector(const Eigen::VectorXd &pose,
                       geometry_msgs::Pose &pose_msg) {
  pose_msg.position.x = pose(0);
  pose_msg.position.y = pose(1);
  pose_msg.position.z = pose(2);

  pose_msg.orientation.w = pose(3);
  pose_msg.orientation.x = pose(4);
  pose_msg.orientation.y = pose(5);
  pose_msg.orientation.z = pose(6);
}

void arrow_initialization(visualization_msgs::Marker &arrow_marker) {

  arrow_marker.type = visualization_msgs::Marker::ARROW;
  arrow_marker.header.frame_id = "world";
  arrow_marker.action = visualization_msgs::Marker::ADD;
  arrow_marker.pose.orientation.w = 1.0;
  arrow_marker.scale.x = 0.005;
  arrow_marker.scale.y = 0.01;
  arrow_marker.scale.z = 0.0;
  arrow_marker.color.r = 1.0;
  arrow_marker.color.b = 0.0;
  arrow_marker.color.g = 0.0;
  arrow_marker.color.a = 1.0;

  arrow_marker.points.resize(2);
}

} // namespace omav_velocity::conversions
