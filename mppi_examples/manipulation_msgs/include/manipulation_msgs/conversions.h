//
// Created by giuseppe on 30.01.21.
//

#pragma once
#include "manipulation_msgs/Input.h"
#include "manipulation_msgs/InputState.h"
#include "manipulation_msgs/State.h"

#include <Eigen/Core>

namespace manipulation::conversions {

void msgToEigen(const manipulation_msgs::State& stateRos,
                Eigen::VectorXd& state);

void eigenToMsg(const Eigen::VectorXd& state, manipulation_msgs::State&);

void msgToEigen(const manipulation_msgs::Input& inputRos,
                Eigen::VectorXd& input);

void eigenToMsg(const Eigen::VectorXd& input, manipulation_msgs::Input&);

void toEigenState(const Eigen::Vector3d& base_pose,
                  const Eigen::Vector3d& base_twist,
                  const Eigen::VectorXd& arm_position,
                  const Eigen::VectorXd& arm_velocity,
                  const double& object_position,
                  const double& object_velocity,
                  const bool& contact_state,
                  const double tank_state,
                  const Eigen::VectorXd& external_torque,
                  Eigen::VectorXd& x);

void fromEigenState(Eigen::Vector3d& base_pose, Eigen::Vector3d& base_twist,
                    Eigen::VectorXd& arm_position,
                    Eigen::VectorXd& arm_velocity, double& object_position,
                    double& object_velocity, bool& contact_state,
                    double tank_state, Eigen::VectorXd& external_torque,
                    const Eigen::VectorXd& x);

void toMsg(const Eigen::Vector3d& base_pose, const Eigen::Vector3d& base_twist,
           const Eigen::VectorXd& arm_position,
           const Eigen::VectorXd& arm_velocity, const double& object_position,
           const double& object_velocity, const bool& contact_state,
           const double tank_state,
           const Eigen::VectorXd& external_torque,
           manipulation_msgs::State&);


}  // namespace manipulation::conversions