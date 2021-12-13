//
// Created by studigem on 30.03.21.
//

#include "mppi_omav_interaction/ros_conversions.h"

namespace omav_interaction::conversions {

void to_trajectory_msg(
    const mppi::observation_array_t &x_opt, const mppi::input_array_t &u_opt,
    const mppi::observation_t &x0_opt,
    trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg) {
  const double dt = 0.015;
  mav_msgs::EigenTrajectoryPoint current_trajectory_point;

  // TODO: WHY 5*VELOCITY?? REMOVE IF WRONG!
  EigenTrajectoryPointFromState(x0_opt, u_opt[0] - 5 * x0_opt.segment<6>(26),
                                current_trajectory_point);

  mav_msgs::EigenTrajectoryPointVector current_trajectory;
  current_trajectory.push_back(current_trajectory_point);
  for (size_t i = 0; i < (x_opt.size() - 6); i++) {
    EigenTrajectoryPointFromStates(x_opt, u_opt, i, current_trajectory_point,
                                   dt);
    current_trajectory.push_back(current_trajectory_point);
  }
  mav_msgs::msgMultiDofJointTrajectoryFromEigen(current_trajectory,
                                                &trajectory_msg);
}

void EigenTrajectoryPointFromStates(
    const observation_array_t &states, const input_array_t &inputs,
    const size_t &i, mav_msgs::EigenTrajectoryPoint &trajectorypoint,
    const double &dt) {
  bool use_input = true;

  trajectorypoint.position_W = states[i].segment<3>(19);
  trajectorypoint.orientation_W_B = Eigen::Quaternion(
      states[i](22), states[i](23), states[i](24), states[i](25));
  trajectorypoint.velocity_W = states[i].segment<3>(26);
  // Angular velocities and accelerations need to be represented in body frame
  trajectorypoint.angular_velocity_W = states[i].segment<3>(29);
  if (!use_input) {
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

    std::vector<double> lin_vel_x, lin_vel_y, lin_vel_z, ang_vel_x, ang_vel_y,
        ang_vel_z;
    OptimalRollouttoVelocityVector(i, 26, states, lin_vel_x);
    OptimalRollouttoVelocityVector(i, 27, states, lin_vel_y);
    OptimalRollouttoVelocityVector(i, 28, states, lin_vel_z);
    OptimalRollouttoVelocityVector(i, 29, states, ang_vel_x);
    OptimalRollouttoVelocityVector(i, 30, states, ang_vel_y);
    OptimalRollouttoVelocityVector(i, 31, states, ang_vel_z);

    double lin_acc_x = filter.filter(lin_vel_x) / dt;
    double lin_acc_y = filter.filter(lin_vel_y) / dt;
    double lin_acc_z = filter.filter(lin_vel_z) / dt;
    double ang_acc_x = filter.filter(ang_vel_x) / dt;
    double ang_acc_y = filter.filter(ang_vel_y) / dt;
    double ang_acc_z = filter.filter(ang_vel_z) / dt;

    Eigen::Matrix<double, 3, 1> linear_acceleration;
    linear_acceleration << lin_acc_x, lin_acc_y, lin_acc_z;

    Eigen::Matrix<double, 3, 1> angular_acceleration_W;
    angular_acceleration_W << ang_acc_x, ang_acc_y, ang_acc_z;
    trajectorypoint.acceleration_W = linear_acceleration;
    trajectorypoint.angular_acceleration_W = angular_acceleration_W;
  } else {
    trajectorypoint.acceleration_W =
        inputs[i + 1].head<3>() - 5 * states[i].segment<3>(26);
    trajectorypoint.angular_acceleration_W =
        inputs[i + 1].segment<3>(3) - 5 * states[i].segment<3>(29);
  }
  trajectorypoint.time_from_start_ns = ros::Duration((i + 1) * dt).toNSec();
}

void PoseStampedMsgFromVector(const Eigen::Matrix<double, 7, 1> &pose,
                              geometry_msgs::PoseStamped &pose_msg) {
  pose_msg.pose.position.x = pose(0);
  pose_msg.pose.position.y = pose(1);
  pose_msg.pose.position.z = pose(2);

  pose_msg.pose.orientation.w = pose(3);
  pose_msg.pose.orientation.x = pose(4);
  pose_msg.pose.orientation.y = pose(5);
  pose_msg.pose.orientation.z = pose(6);
}

void PoseMsgFromVector(const Eigen::Matrix<double, 7, 1> &pose,
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

void RPYtoQuaterniond(const double &roll, const double &pitch,
                      const double &yaw, Eigen::Quaterniond &q) {
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

void OptimalRollouttoVelocityVector(const int trajectory_point_index,
                                    const int velocity_index,
                                    const observation_array_t &states,
                                    std::vector<double> &velocity_vector) {
  velocity_vector = {states[trajectory_point_index](velocity_index),
                     states[trajectory_point_index + 1](velocity_index),
                     states[trajectory_point_index + 2](velocity_index),
                     states[trajectory_point_index + 3](velocity_index),
                     states[trajectory_point_index + 4](velocity_index),
                     states[trajectory_point_index + 5](velocity_index),
                     states[trajectory_point_index + 6](velocity_index)};
}

void InterpolateTrajectoryPoints(
    const trajectory_msgs::MultiDOFJointTrajectoryPoint &trajectory_msg_point_1,
    const trajectory_msgs::MultiDOFJointTrajectoryPoint &trajectory_msg_point_2,
    mav_msgs::EigenTrajectoryPoint *trajectory_point) {
  double t = 0.01 / 0.015;

  Eigen::Vector3d position_2 = mav_msgs::vector3FromMsg(
      trajectory_msg_point_2.transforms[0].translation);
  Eigen::Vector3d position_1 = mav_msgs::vector3FromMsg(
      trajectory_msg_point_1.transforms[0].translation);

  trajectory_point->position_W = position_1 + (position_2 - position_1) * t;

  Eigen::Vector3d velocity_2 =
      mav_msgs::vector3FromMsg(trajectory_msg_point_2.velocities[0].linear);
  Eigen::Vector3d velocity_1 =
      mav_msgs::vector3FromMsg(trajectory_msg_point_1.velocities[0].linear);

  trajectory_point->velocity_W = velocity_1 + (velocity_2 - velocity_1) * t;

  Eigen::Vector3d acceleration_2 =
      mav_msgs::vector3FromMsg(trajectory_msg_point_2.accelerations[0].linear);
  Eigen::Vector3d acceleration_1 =
      mav_msgs::vector3FromMsg(trajectory_msg_point_1.accelerations[0].linear);

  trajectory_point->acceleration_W =
      acceleration_1 + (acceleration_2 - acceleration_1) * t;

  // Slerp for quaternion interpolation!
  Eigen::Quaterniond orientation_2(
      trajectory_msg_point_2.transforms[0].rotation.w,
      trajectory_msg_point_2.transforms[0].rotation.x,
      trajectory_msg_point_2.transforms[0].rotation.y,
      trajectory_msg_point_2.transforms[0].rotation.z);

  Eigen::Quaterniond orientation_1(
      trajectory_msg_point_1.transforms[0].rotation.w,
      trajectory_msg_point_1.transforms[0].rotation.x,
      trajectory_msg_point_1.transforms[0].rotation.y,
      trajectory_msg_point_1.transforms[0].rotation.z);

  trajectory_point->orientation_W_B = orientation_1.slerp(t, orientation_2);

  Eigen::Vector3d angular_velocity_2 =
      mav_msgs::vector3FromMsg(trajectory_msg_point_2.velocities[0].angular);
  Eigen::Vector3d angular_velocity_1 =
      mav_msgs::vector3FromMsg(trajectory_msg_point_1.velocities[0].angular);

  trajectory_point->angular_velocity_W =
      angular_velocity_1 + (angular_velocity_2 - angular_velocity_1) * t;

  Eigen::Vector3d angular_acceleration_2 =
      mav_msgs::vector3FromMsg(trajectory_msg_point_2.accelerations[0].angular);
  Eigen::Vector3d angular_acceleration_1 =
      mav_msgs::vector3FromMsg(trajectory_msg_point_1.accelerations[0].angular);

  trajectory_point->angular_acceleration_W =
      angular_acceleration_1 +
      (angular_acceleration_2 - angular_acceleration_1) * t;
}

/**
 * @brief      Transforms state and input to trajectory point. Input containts
 *             accelerations, state containts pose and velocities.
 *
 * @param[in]  state            The state
 * @param[in]  input            The input
 * @param      trajectorypoint  The trajectorypoint
 */
void EigenTrajectoryPointFromState(
    const observation_t &state, const input_t &input,
    mav_msgs::EigenTrajectoryPoint &trajectorypoint) {
  trajectorypoint.position_W = state.segment<3>(19);
  trajectorypoint.orientation_W_B =
      Eigen::Quaternion(state(22), state(23), state(24), state(25));
  trajectorypoint.velocity_W = state.segment<3>(26);
  trajectorypoint.angular_velocity_W = state.segment<3>(29);
  trajectorypoint.acceleration_W = input.head(3);
  trajectorypoint.angular_acceleration_W = input.segment<3>(3);
}

// ToDo: Remove this as soon as possible, because it is crazy ugly @Matthias
void PoseMsgForVelocityFromVector(const Eigen::Vector3d &velocity,
                                  geometry_msgs::Pose &pose_msg) {
  pose_msg.position.x = velocity(0);
  pose_msg.position.y = velocity(1);
  pose_msg.position.z = velocity(2);
}

}  // namespace omav_interaction::conversions
