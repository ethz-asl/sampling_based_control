//
// Created by giuseppe on 30.01.21.
//

#include "manipulation_msgs/conversions.h"

namespace manipulation::conversions {

void msgToEigen(const manipulation_msgs::State &stateRos,
                Eigen::VectorXd &state, double &time) {
  time = stateRos.header.stamp.toSec();
  state.resize(manipulation_msgs::State::SIZE);
  state(0) = stateRos.base_pose.x;
  state(1) = stateRos.base_pose.y;
  state(2) = stateRos.base_pose.z;
  state(12) = stateRos.base_twist.linear.x;
  state(13) = stateRos.base_twist.linear.y;
  state(14) = stateRos.base_twist.angular.z;

  state(24) = stateRos.object_position;
  state(25) = stateRos.object_velocity;
  state(26) = stateRos.in_contact;
  state(27) = stateRos.tank_state;
  state(28) = stateRos.base_effort.x;
  state(29) = stateRos.base_effort.y;
  state(30) = stateRos.base_effort.z;

  for (size_t i = 0; i < 9; i++) {
    state(3 + i) = stateRos.arm_state.position[i];
    state(15 + i) = stateRos.arm_state.velocity[i];
    state(31 + i) = stateRos.arm_state.effort[i];  // indeed tau ext
  }
}

void eigenToMsg(const Eigen::VectorXd &state, const double &time,
                manipulation_msgs::State &stateRos) {
  stateRos.header.stamp = ros::Time().fromSec(time);
  stateRos.base_pose.x = state(0);
  stateRos.base_pose.y = state(1);
  stateRos.base_pose.z = state(2);
  stateRos.base_twist.linear.x = state(12);
  stateRos.base_twist.linear.y = state(13);
  stateRos.base_twist.angular.z = state(14);

  stateRos.object_position = state(24);
  stateRos.object_velocity = state(25);
  stateRos.in_contact = state(26);
  stateRos.tank_state = state(27);
  stateRos.base_effort.x = state(28);
  stateRos.base_effort.y = state(29);
  stateRos.base_effort.z = state(30);

  stateRos.arm_state.position.resize(9);
  stateRos.arm_state.velocity.resize(9);
  stateRos.arm_state.effort.resize(9);
  for (size_t i = 0; i < 9; i++) {
    stateRos.arm_state.position[i] = state(3 + i);
    stateRos.arm_state.velocity[i] = state(15 + i);
    stateRos.arm_state.effort[i] = state(31 + i);  // indeed tau ext
  }
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
                manipulation_msgs::InputState &){};

void toEigenState(const Eigen::Vector3d &base_pose,
                  const Eigen::Vector3d &base_twist,
                  const Eigen::Vector3d &base_effort,
                  const Eigen::VectorXd &arm_position,
                  const Eigen::VectorXd &arm_velocity,
                  const Eigen::VectorXd &arm_effort,
                  const double &object_position,
                  const double &object_velocity,
                  const bool &contact_state,
                  const double tank_state,
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
    state(31 + i) = arm_effort(i);
  }

  state(24) = object_position;
  state(25) = object_velocity;
  state(26) = contact_state;
  state(27) = tank_state;
  state(28) = base_effort.x();
  state(29) = base_effort.x();
  state(30) = base_effort.x();
}

void fromEigenState(Eigen::Vector3d &base_pose,
                    Eigen::Vector3d &base_twist,
                    Eigen::Vector3d &base_effort,
                    Eigen::VectorXd &arm_position,
                    Eigen::VectorXd &arm_velocity,
                    Eigen::VectorXd &arm_effort,
                    double &object_position,
                    double &object_velocity,
                    bool &contact_state,
                    double &tank_state,
                    const Eigen::VectorXd &state) {
  assert(state.size() == manipulation_msgs::State::SIZE);
  base_pose.x() = state(0);
  base_pose.y() = state(1);
  base_pose.z() = state(2);
  base_twist.x() = state(12);
  base_twist.y() = state(13);
  base_twist.z() = state(14);
  base_effort.x() = state(28);
  base_effort.y() = state(29);
  base_effort.z() = state(30);

  for (int i = 0; i < 9; i++) {
    arm_position(i) = state(3 + i);
    arm_velocity(i) = state(15 + i);
    arm_effort(i) = state(31 + i);
  }

  object_position = state(24);
  object_velocity = state(25);
  contact_state = state(26);
  tank_state = state(27);
}

void toMsg(const double &time,
           const Eigen::Vector3d &base_pose,
           const Eigen::Vector3d &base_twist,
           const Eigen::Vector3d &base_effort,
           const Eigen::VectorXd &arm_position,
           const Eigen::VectorXd &arm_velocity,
           const Eigen::VectorXd &arm_effort,
           const double &object_position,
           const double &object_velocity,
           const bool &contact_state,
           const double &tank_state,
           manipulation_msgs::State &stateRos) {
  stateRos.header.stamp = ros::Time().fromSec(time);
  stateRos.base_pose.x = base_pose.x();
  stateRos.base_pose.y = base_pose.y();
  stateRos.base_pose.z = base_pose.z();
  stateRos.base_twist.linear.x = base_twist.x();
  stateRos.base_twist.linear.y = base_twist.y();
  stateRos.base_twist.angular.z = base_twist.z();
  stateRos.base_effort.x = base_effort.x();
  stateRos.base_effort.y = base_effort.y();
  stateRos.base_effort.z = base_effort.z();

  stateRos.arm_state.position.resize(9);
  stateRos.arm_state.velocity.resize(9);
  stateRos.arm_state.effort.resize(9);
  for (int i = 0; i < 9; i++) {
    stateRos.arm_state.position[i] = arm_position(i);
    stateRos.arm_state.velocity[i] = arm_velocity(i);
    stateRos.arm_state.effort[i] = arm_effort(i);
  }
  stateRos.object_position = object_position;
  stateRos.object_velocity = object_velocity;
  stateRos.in_contact = contact_state;
  stateRos.tank_state = tank_state;
}

std::string eigenToString(const Eigen::VectorXd &x) {
  std::stringstream ss;
  ss << "base position =" << x.head<3>().transpose() << std::endl;
  ss << "base twist    =" << x.segment<3>(12).transpose() << std::endl;
  ss << "arm position  =" << x.segment<7>(3).transpose() << std::endl;
  ss << "arm velocity  =" << x.segment<7>(15).transpose() << std::endl;
  ss << "fing position =" << x.segment<2>(10).transpose() << std::endl;
  ss << "fing velocity =" << x.segment<2>(22).transpose() << std::endl;
  ss << "object state  =" << x.segment<2>(24).transpose() << std::endl;
  ss << "contact state =" << x.segment<1>(26).transpose() << std::endl;
  ss << "tank state    =" << x(27) << std::endl;
  ss << "ext tau       =" << x.segment<12>(27).transpose() << std::endl;
  return ss.str();
}

}  // namespace manipulation::conversions