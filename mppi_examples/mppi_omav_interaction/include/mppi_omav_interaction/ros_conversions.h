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
    const mppi::observation_array_t &x_opt,
    trajectory_msgs::MultiDOFJointTrajectory &trajectory_msg);

void EigenTrajectoryPointFromState(
    const mppi::observation_array_t &state, int i,
    mav_msgs::EigenTrajectoryPoint &trajectorypoint, double dt);

void PoseStampedMsgFromVector(const Eigen::VectorXd &pose,
                              geometry_msgs::PoseStamped &pose_msg);
void PoseMsgFromVector(const Eigen::VectorXd &pose,
                       geometry_msgs::Pose &pose_msg);
void arrow_initialization(visualization_msgs::Marker &arrow_marker);

} // namespace omav_velocity::conversions
