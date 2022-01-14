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

/**
 * @brief      Transform optimal rollout to trajectory message
 *
 * @param[in]  x_opt           States of optimal rollout
 * @param[in]  u_opt           Inputs of optimal rollout
 * @param[in]  x0_opt          Initial state
 * @param      trajectory_msg  The trajectory message
 */
void to_trajectory_msg(
    const mppi::observation_array_t &x_opt, const mppi::input_array_t &u_opt,
    const mppi::observation_t &x0_opt,
    trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg);
void to_trajectory_msg(
    const mppi::observation_array_t &x_opt, const mppi::input_array_t &u_opt,
    const std::vector<double> &tt,
    trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg);

void EigenTrajectoryPointFromStates(
    const mppi::observation_array_t &state, const mppi::input_array_t &input,
    const size_t &i, mav_msgs::EigenTrajectoryPoint &trajectorypoint,
    const double &dt);

/**
 * @brief      Transforms omav_interaction state to reference point
 *
 * @param[in]  state            The state
 * @param[in]  input            The input
 * @param      trajectorypoint  The trajectorypoint
 */
void EigenTrajectoryPointFromState(
    const mppi::observation_t &state, const mppi::input_t &input,
    mav_msgs::EigenTrajectoryPoint &trajectorypoint);
void EigenTrajectoryPointFromState(
    const observation_t &state, const input_t &input,
    const int64_t &time_from_start_ns,
    mav_msgs::EigenTrajectoryPoint &trajectorypoint);

void PoseStampedMsgFromVector(const Eigen::Matrix<double, 7, 1> &pose,
                              geometry_msgs::PoseStamped &pose_msg);
void PoseMsgFromVector(const Eigen::Matrix<double, 7, 1> &pose,
                       geometry_msgs::Pose &pose_msg);
void PoseMsgForVelocityFromVector(const Eigen::Vector3d &velocity,
                                  geometry_msgs::Pose &pose_msg);

void arrow_initialization(visualization_msgs::Marker &arrow_marker);
// Roll, Pitch and Yaw in deg
void RPYtoQuaterniond(const double &roll, const double &pitch,
                      const double &yaw, Eigen::Quaterniond &q);
// Velocity Vector from optimal rollout
void OptimalRollouttoVelocityVector(const int trajectory_point_index,
                                    const int velocity_index,
                                    const observation_array_t &states,
                                    std::vector<double> &velocity_vector);
void MultiDofJointTrajectoryPointFromState(
    const observation_t &state,
    trajectory_msgs::MultiDOFJointTrajectoryPoint &point);
// Interpolate trajecotry points
void InterpolateTrajectoryPoints(
    const trajectory_msgs::MultiDOFJointTrajectoryPoint &trajectory_msg_point_1,
    const trajectory_msgs::MultiDOFJointTrajectoryPoint &trajectory_msg_point_2,
    const double &t,
    mav_msgs::EigenTrajectoryPoint *trajectory_point);

}  // namespace omav_interaction::conversions
