//
// Created by studigem on 30.03.21.
//

#pragma once

#include "mppi_ros/controller_interface.h"

#include <mav_msgs/conversions.h>
#include <mppi/filters/gram_savitzky_golay.h>
#include <std_msgs/Duration.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <visualization_msgs/Marker.h>

namespace omav_interaction::conversions {
void to_trajectory_msg(
    const mppi::observation_array_t &x_opt, const mppi::input_array_t &u_opt,
    const mppi::observation_t &x0_opt,
    trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg);

void EigenTrajectoryPointFromStates(
    const mppi::observation_array_t &state, const mppi::input_array_t &input,
    int i, mav_msgs::EigenTrajectoryPoint &trajectorypoint, double dt);

void EigenTrajectoryPointFromState(
    const mppi::observation_t &state, const mppi::input_t &input,
    mav_msgs::EigenTrajectoryPoint &trajectorypoint);

void PoseStampedMsgFromVector(const Eigen::Matrix<double, 7, 1> &pose,
                              geometry_msgs::PoseStamped &pose_msg);
void PoseMsgFromVector(const Eigen::Matrix<double, 7, 1> &pose,
                       geometry_msgs::Pose &pose_msg);
void PoseMsgForVelocityFromVector(const Eigen::Vector3d &velocity,
                                  geometry_msgs::Pose &pose_msg);

void arrow_initialization(visualization_msgs::Marker &arrow_marker);
// Roll, Pitch and Yaw in deg
void RPYtoQuaterniond(double roll, double pitch, double yaw,
                      Eigen::Quaterniond &q);
// Velocity Vector from optimal rollout
void OptimalRollouttoVelocityVector(const int trajectory_point_index,
                                    const int velocity_index,
                                    const observation_array_t &states,
                                    std::vector<double> &velocity_vector);
// Interpolate trajecotry points
void InterpolateTrajectoryPoints(
    const trajectory_msgs::MultiDOFJointTrajectoryPoint &trajectory_msg_point_1,
    const trajectory_msgs::MultiDOFJointTrajectoryPoint &trajectory_msg_point_2,
    mav_msgs::EigenTrajectoryPoint *trajectory_point);

} // namespace omav_velocity::conversions
