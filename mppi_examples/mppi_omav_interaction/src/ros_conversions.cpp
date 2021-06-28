//
// Created by studigem on 30.03.21.
//

#include "mppi_omav_interaction/ros_conversions.h"

namespace omav_interaction::conversions {
void to_trajectory_msg(
    const mppi::observation_array_t &x_opt, const mppi::input_array_t &u_opt,
    trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg) {
  double dt = 0.015;
  mav_msgs::EigenTrajectoryPoint current_trajectory_point;
  mav_msgs::EigenTrajectoryPointVector current_trajectory;

  for (int i = 0; i < (x_opt.size() - 6); i++) {
    EigenTrajectoryPointFromState(x_opt, u_opt, i, current_trajectory_point,
                                  dt);
    current_trajectory.push_back(current_trajectory_point);
  }
  mav_msgs::msgMultiDofJointTrajectoryFromEigen(current_trajectory,
                                                &trajectory_msg);
}

void EigenTrajectoryPointFromState(
    const observation_array_t &states, const input_array_t &inputs, int i,
    mav_msgs::EigenTrajectoryPoint &trajectorypoint, double dt) {

  trajectorypoint.position_W = states[i].head<3>();
  trajectorypoint.orientation_W_B =
      Eigen::Quaternion(states[i](3), states[i](4), states[i](5), states[i](6));
  trajectorypoint.velocity_W = inputs[i].head<3>();
  Eigen::Matrix3d R_W_B_des =
      trajectorypoint.orientation_W_B.toRotationMatrix();
  // Angluar velocities and accelerations need to be represented in body frame
  trajectorypoint.angular_velocity_W = inputs[i].tail<3>();
  // Filter the velocities to calculate the desired accelerations
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
      inputs[i](0),     inputs[i + 1](0), inputs[i + 2](0), inputs[i + 3](0),
      inputs[i + 4](0), inputs[i + 5](0), inputs[i + 6](0)};
  std::vector<double> lin_vel_y = {
      inputs[i](1),     inputs[i + 1](1), inputs[i + 2](1), inputs[i + 3](1),
      inputs[i + 4](1), inputs[i + 5](1), inputs[i + 6](1)};
  std::vector<double> lin_vel_z = {
      inputs[i](2),     inputs[i + 1](2), inputs[i + 2](2), inputs[i + 3](2),
      inputs[i + 4](2), inputs[i + 5](2), inputs[i + 6](2)};
  std::vector<double> ang_vel_x = {
      inputs[i](3),     inputs[i + 1](3), inputs[i + 2](3), inputs[i + 3](3),
      inputs[i + 4](3), inputs[i + 5](3), inputs[i + 6](3)};
  std::vector<double> ang_vel_y = {
      inputs[i](4),     inputs[i + 1](4), inputs[i + 2](4), inputs[i + 3](4),
      inputs[i + 4](4), inputs[i + 5](4), inputs[i + 6](4)};
  std::vector<double> ang_vel_z = {
      inputs[i](5),     inputs[i + 1](5), inputs[i + 2](5), inputs[i + 3](5),
      inputs[i + 4](5), inputs[i + 5](5), inputs[i + 6](5)};

  double lin_acc_x = filter.filter(lin_vel_x);
  double lin_acc_y = filter.filter(lin_vel_y);
  double lin_acc_z = filter.filter(lin_vel_z);
  double ang_acc_x = filter.filter(ang_vel_x);
  double ang_acc_y = filter.filter(ang_vel_y);
  double ang_acc_z = filter.filter(ang_vel_z);

  Eigen::Matrix<double, 3, 1> linear_acceleration;
  linear_acceleration << lin_acc_x / dt, lin_acc_y / dt, lin_acc_z / dt;

  Eigen::Matrix<double, 3, 1> angular_acceleration_W;
  angular_acceleration_W << ang_acc_x / dt, ang_acc_y / dt, ang_acc_z / dt;

  trajectorypoint.acceleration_W = linear_acceleration;
  trajectorypoint.angular_acceleration_W = angular_acceleration_W;
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

void RPYtoQuaterniond(double roll, double pitch, double yaw,
                      Eigen::Quaterniond &q) {
  // convert deg to rad
  double roll_rad = roll * 2 * M_PI / 360;
  double pitch_rad = pitch * 2 * M_PI / 360;
  double yaw_rad = yaw * 2 * M_PI / 360;

  double cy = cos(yaw_rad * 0.5);
  double sy = sin(yaw_rad * 0.5);
  double cp = cos(pitch_rad * 0.5);
  double sp = sin(pitch_rad * 0.5);
  double cr = cos(roll_rad * 0.5);
  double sr = sin(roll_rad * 0.5);

  q.w() = cr * cp * cy + sr * sp * sy;
  q.x() = sr * cp * cy - cr * sp * sy;
  q.y() = cr * sp * cy + sr * cp * sy;
  q.z() = cr * cp * sy - sr * sp * cy;
}

} // namespace omav_velocity::conversions
