//
// Created by giuseppe on 30.01.21.
//

#pragma once
#include "manipulation_msgs/Input.h"
#include "manipulation_msgs/InputState.h"
#include "manipulation_msgs/State.h"

#include <Eigen/Core>

namespace manipulation::conversions {

// clang-format off
void msgToEigen(const manipulation_msgs::State& stateRos,
                Eigen::VectorXd& state, double& time);

void eigenToMsg(const Eigen::VectorXd& state, const double& time,
                manipulation_msgs::State&);

void msgToEigen(const manipulation_msgs::Input& inputRos,
                Eigen::VectorXd& input);

void eigenToMsg(const Eigen::VectorXd& input, manipulation_msgs::Input&);

void toEigenState(const Eigen::Vector3d& base_pose,
                  const Eigen::Vector3d& base_twist,
                  const Eigen::Vector3d& base_effort,
                  const Eigen::VectorXd& arm_position,
                  const Eigen::VectorXd& arm_velocity,
                  const Eigen::VectorXd& arm_effort,
                  const double& object_position,
                  const double& object_velocity,
                  const bool& contact_state,
                  const double tank_state,
                  Eigen::VectorXd& x);

void fromEigenState(Eigen::Vector3d& base_pose,
                    Eigen::Vector3d& base_twist,
                    Eigen::Vector3d& base_effort,
                    Eigen::VectorXd& arm_position,
                    Eigen::VectorXd& arm_velocity,
                    Eigen::VectorXd& arm_effort,
                    double& object_position,
                    double& object_velocity,
                    bool& contact_state,
                    double& tank_state,
                    const Eigen::VectorXd& x);

void toMsg(const double& time,
           const Eigen::Vector3d& base_pose,
           const Eigen::Vector3d& base_twist,
           const Eigen::Vector3d& base_effort,
           const Eigen::VectorXd& arm_position,
           const Eigen::VectorXd& arm_velocity,
           const Eigen::VectorXd& arm_effort,
           const double& object_position,
           const double& object_velocity,
           const bool& contact_state,
           const double& tank_state,
           manipulation_msgs::State&);

std::string eigenToString(const Eigen::VectorXd& x);


//clang-format on
}  // namespace manipulation::conversions