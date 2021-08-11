//
// Created by giuseppe on 30.01.21.
//

#include "manipulation_msgs/conversions.h"

namespace manipulation::conversions {

void msgToEigen(const manipulation_msgs::State &stateRos,
                Eigen::VectorXd &state) {
  state.resize(manipulation_msgs::State::SIZE);

  state(0) = stateRos.base_pose.x;
  state(1) = stateRos.base_pose.y;
  state(2) = stateRos.base_pose.z;
  state(12) = stateRos.base_twist.linear.x;
  state(13) = stateRos.base_twist.linear.y;
  state(14) = stateRos.base_twist.angular.z;

  for (size_t i = 0; i < 9; i++) {
    state(3 + i) = stateRos.arm_state.position[i];
    state(15 + i) = stateRos.arm_state.velocity[i];
    state(31 + i) = stateRos.arm_state.effort[i];  // indeed tau ext
  }

  state(24) = stateRos.object_position;
  state(25) = stateRos.object_velocity;
  state(26) = stateRos.in_contact;
  state(27) = stateRos.tank_state;
}

void eigenToMsg(const Eigen::VectorXd &state,
                manipulation_msgs::State &stateRos) {
  stateRos.base_pose.x = state(0);
  stateRos.base_pose.y = state(1);
  stateRos.base_pose.z = state(2);
  stateRos.base_twist.linear.x = state(12);
  stateRos.base_twist.linear.y = state(13);
  stateRos.base_twist.angular.z = state(14);

  stateRos.arm_state.position.resize(9);
  stateRos.arm_state.velocity.resize(9);
  stateRos.arm_state.effort.resize(9);

  for (size_t i = 0; i < 9; i++) {
    stateRos.arm_state.position[i] = state(3 + i);
    stateRos.arm_state.velocity[i] = state(15 + i);
    stateRos.arm_state.effort[i] = state(31 + i);  // indeed tau ext
  }
  stateRos.object_position = state(24);
  stateRos.object_velocity = state(25);
  stateRos.in_contact = state(26);
  stateRos.tank_state = state(27);
}

void msgToEigen(const manipulation_msgs::Input &inputRos,
                Eigen::VectorXd &input) {
  input = Eigen::VectorXd::Zero(manipulation_msgs::Input::SIZE);
  input(0) = inputRos.base_twist.linear.x;
  input(1) = inputRos.base_twist.linear.y;
  input(2) = inputRos.base_twist.angular.z;
  for (size_t i = 0; i < 7; i++) {
    input(3 + i) = inputRos.joint_velocities[i];  // gripper stays at zero
  }
}

void eigenToMsg(const Eigen::VectorXd &input,
                manipulation_msgs::Input &inputRos) {
  inputRos.base_twist.linear.x = input(0);
  inputRos.base_twist.linear.x = input(1);
  inputRos.base_twist.linear.x = input(2);
  inputRos.joint_velocities.resize(manipulation_msgs::Input::SIZE);
  for (size_t i = 0; i < 8; i++) {
    inputRos.joint_velocities[i] = input(3 + i);
  }
}

void eigenToMsg(const Eigen::VectorXd &inputState,
                manipulation_msgs::InputState &);

void toEigenState(const Eigen::Vector3d &base_pose,
                  const Eigen::Vector3d &base_twist,
                  const Eigen::VectorXd &arm_position,
                  const Eigen::VectorXd &arm_velocity,
                  const double &object_position, const double &object_velocity,
                  const bool &contact_state,
                  const double tank_state,
                  const Eigen::VectorXd &external_torque,
                  Eigen::VectorXd &state) {
  state.resize(manipulation_msgs::State::SIZE);

  state(0) = base_pose.x();
  state(1) = base_pose.y();
  state(2) = base_pose.z();
  state(12) = base_twist.x();
  state(13) = base_twist.y();
  state(14) = base_twist.z();

  for (size_t i = 0; i < 9; i++) {
    state(3 + i) = arm_position(i);
    state(15 + i) = arm_velocity(i);
    state(31 + i) = external_torque(3+i);
  }

  state(24) = object_position;
  state(25) = object_velocity;
  state(26) = contact_state;
  state(27) = tank_state;
}

void toMsg(const Eigen::Vector3d &base_pose, const Eigen::Vector3d &base_twist,
           const Eigen::VectorXd &arm_position,
           const Eigen::VectorXd &arm_velocity, const double &object_position,
           const double &object_velocity, const bool &contact_state,
           const double& tank_state, const Eigen::VectorXd& external_torque,
           manipulation_msgs::State &stateRos) {
  stateRos.base_pose.x = base_pose.x();
  stateRos.base_pose.y = base_pose.y();
  stateRos.base_pose.z = base_pose.z();
  stateRos.base_twist.linear.x = base_twist.x();
  stateRos.base_twist.linear.y = base_twist.y();
  stateRos.base_twist.angular.z = base_twist.z();

  stateRos.arm_state.position.resize(9);
  stateRos.arm_state.velocity.resize(9);
  for (size_t i = 0; i < 9; i++) {
    stateRos.arm_state.position[i] = arm_position(i);
    stateRos.arm_state.velocity[i] = arm_velocity(i);
    stateRos.arm_state.effort[i] = external_torque(3+i);
  }
  stateRos.object_position = object_position;
  stateRos.object_velocity = object_velocity;
  stateRos.in_contact = contact_state;
  stateRos.tank_state = tank_state;
}

}  // namespace manipulation::conversions