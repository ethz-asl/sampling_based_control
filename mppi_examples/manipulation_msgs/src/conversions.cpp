//
// Created by giuseppe on 30.01.21.
//

#include "manipulation_msgs/conversions.h"

namespace manipulation::conversions {

void msgToEigen(const manipulation_msgs::State &stateRos,
                Eigen::VectorXd &state) {
  if (stateRos.mode == manipulation_msgs::State::FIXED_BASE) {
    state.resize(manipulation_msgs::State::FIXED_BASE_SIZE);

    for (size_t i = 0; i < 9; i++) {
      state(i) = stateRos.arm_state.position[i];
      state(9 + i) = stateRos.arm_state.velocity[i];
    }
    state(18) = stateRos.object_position;
    state(19) = stateRos.object_velocity;
    state(20) = stateRos.in_contact;
  } else {
    state.resize(manipulation_msgs::State::MOVING_BASE_SIZE);

    state(0) = stateRos.base_pose.x;
    state(1) = stateRos.base_pose.y;
    state(2) = stateRos.base_pose.z;
    state(12) = stateRos.base_twist.linear.x;
    state(13) = stateRos.base_twist.linear.y;
    state(14) = stateRos.base_twist.angular.z;

    for (size_t i = 0; i < 9; i++) {
      state(3 + i) = stateRos.arm_state.position[i];
      state(15 + i) = stateRos.arm_state.velocity[i];
    }

    state(24) = stateRos.object_position;
    state(25) = stateRos.object_velocity;
    state(26) = stateRos.in_contact;
  }
}

void eigenToMsg(const Eigen::VectorXd &state,
                manipulation_msgs::State &stateRos) {
  if (state.size() == manipulation_msgs::State::FIXED_BASE_SIZE) {
    stateRos.mode = manipulation_msgs::State::FIXED_BASE;

    stateRos.arm_state.position.resize(9);
    stateRos.arm_state.velocity.resize(9);
    for (size_t i = 0; i < 9; i++) {
      stateRos.arm_state.position[i] = state(i);
      stateRos.arm_state.velocity[i] = state(9 + i);
    }
    stateRos.object_position = state(18);
    stateRos.object_velocity = state(19);
    stateRos.in_contact = state(20);
  } else if (state.size() == manipulation_msgs::State::MOVING_BASE_SIZE) {
    stateRos.mode = manipulation_msgs::State::MOVING_BASE;

    stateRos.base_pose.x = state(0);
    stateRos.base_pose.y = state(1);
    stateRos.base_pose.z = state(2);
    stateRos.base_twist.linear.x = state(12);
    stateRos.base_twist.linear.y = state(13);
    stateRos.base_twist.angular.z = state(14);

    stateRos.arm_state.position.resize(9);
    stateRos.arm_state.velocity.resize(9);
    for (size_t i = 0; i < 9; i++) {
      stateRos.arm_state.position[i] = state(3 + i);
      stateRos.arm_state.velocity[i] = state(15 + i);
    }
    stateRos.object_position = state(24);
    stateRos.object_velocity = state(25);
    stateRos.in_contact = state(26);
  } else {
    throw std::runtime_error("[eigenToMsg]: eigen state has wrong size: " +
                             std::to_string(state.size()));
  }
}

void msgToEigen(const manipulation_msgs::Input &inputRos,
                Eigen::VectorXd &input) {
  if (inputRos.mode == manipulation_msgs::Input::FIXED_BASE) {
    input = Eigen::VectorXd::Zero(manipulation_msgs::Input::FIXED_BASE_SIZE);
    for (size_t i = 0; i < 7; i++)
      input(i) = inputRos.joint_velocities[i];  // gripper stays at zero
  } else {
    input = Eigen::VectorXd::Zero(manipulation_msgs::Input::MOVING_BASE_SIZE);
    input(0) = inputRos.base_twist.linear.x;
    input(1) = inputRos.base_twist.linear.y;
    input(2) = inputRos.base_twist.angular.z;
    for (size_t i = 0; i < 7; i++)
      input(3 + i) = inputRos.joint_velocities[i];  // gripper stays at zero
  }
}

void eigenToMsg(const Eigen::VectorXd &input,
                manipulation_msgs::Input &inputRos) {
  if (input.size() == manipulation_msgs::Input::FIXED_BASE_SIZE) {
    inputRos.joint_velocities.resize(manipulation_msgs::Input::FIXED_BASE_SIZE);
    for (size_t i = 0; i < 8; i++) inputRos.joint_velocities[i] = input(i);
  } else if (input.size() == manipulation_msgs::Input::MOVING_BASE_SIZE) {
    inputRos.base_twist.linear.x = input(0);
    inputRos.base_twist.linear.x = input(1);
    inputRos.base_twist.linear.x = input(2);
    inputRos.joint_velocities.resize(
        manipulation_msgs::Input::MOVING_BASE_SIZE);
    for (size_t i = 0; i < 8; i++) inputRos.joint_velocities[i] = input(3 + i);
  }
}

void eigenToMsg(const Eigen::VectorXd &inputState,
                manipulation_msgs::InputState &);

void toEigenState(const Eigen::Vector3d &base_pose,
                  const Eigen::Vector3d &base_twist,
                  const Eigen::VectorXd &arm_position,
                  const Eigen::VectorXd &arm_velocity,
                  const double &object_position, const double &object_velocity,
                  const bool &contact_state, Eigen::VectorXd &state) {
  state.resize(manipulation_msgs::State::MOVING_BASE_SIZE);

  state(0) = base_pose.x();
  state(1) = base_pose.y();
  state(2) = base_pose.z();
  state(12) = base_twist.x();
  state(13) = base_twist.y();
  state(14) = base_twist.z();

  for (size_t i = 0; i < 9; i++) {
    state(3 + i) = arm_position(i);
    state(15 + i) = arm_velocity(i);
  }

  state(24) = object_position;
  state(25) = object_velocity;
  state(26) = contact_state;
}

void toMsg(const Eigen::Vector3d &base_pose, const Eigen::Vector3d &base_twist,
           const Eigen::VectorXd &arm_position,
           const Eigen::VectorXd &arm_velocity, const double &object_position,
           const double &object_velocity, const bool &contact_state,
           manipulation_msgs::State &stateRos) {
  stateRos.mode = manipulation_msgs::State::MOVING_BASE;

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
  }
  stateRos.object_position = object_position;
  stateRos.object_velocity = object_velocity;
  stateRos.in_contact = contact_state;
}

void toEigenState(const Eigen::VectorXd &arm_position,
                  const Eigen::VectorXd &arm_velocity,
                  const double &object_position, const double &object_velocity,
                  const bool &contact_state, Eigen::VectorXd &state) {
  state.resize(manipulation_msgs::State::FIXED_BASE_SIZE);

  for (size_t i = 0; i < 9; i++) {
    state(i) = arm_position(i);
    state(9 + i) = arm_velocity(i);
  }

  state(18) = object_position;
  state(19) = object_velocity;
  state(20) = contact_state;
}

void toMsg(const Eigen::VectorXd &arm_position,
           const Eigen::VectorXd &arm_velocity, const double &object_position,
           const double &object_velocity, const bool &contact_state,
           manipulation_msgs::State &stateRos) {
  stateRos.mode = manipulation_msgs::State::FIXED_BASE;

  stateRos.arm_state.position.resize(9);
  stateRos.arm_state.velocity.resize(9);
  for (size_t i = 0; i < 9; i++) {
    stateRos.arm_state.position[i] = arm_position(i);
    stateRos.arm_state.velocity[i] = arm_velocity(i);
  }
  stateRos.object_position = object_position;
  stateRos.object_velocity = object_velocity;
  stateRos.in_contact = contact_state;
}
}  // namespace manipulation::conversions