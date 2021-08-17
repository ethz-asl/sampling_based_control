//
// Created by giuseppe on 09.08.21.
//

#include "mppi_manipulation/params/gains.h"

using namespace manipulation;

bool PIDGains::init_from_ros(const ros::NodeHandle& nh, const std::string& prefix) {
  if (!base_gains.parse_from_ros(nh, prefix + "dynamics/gains/base_gains")) return false;
  if (!arm_gains.parse_from_ros(nh, prefix + "dynamics/gains/arm_gains")) return false;
  if (!gripper_gains.parse_from_ros(nh, prefix + "dynamics/gains/gripper_gains"))
    return false;
  return true;
}

std::ostream& operator<<(std::ostream& os, const manipulation::PIDGains& gains){
  os << "Gains:" << std::endl;
  os << "base gains: " << std::endl;
  os << " - kp=[" << gains.base_gains.Kp.transpose() << "]" << std::endl;
  os << " - kd=[" << gains.base_gains.Kd.transpose() << "]" << std::endl;
  os << " - ki=[" << gains.base_gains.Ki.transpose() << "]" << std::endl;
  os << " - i_max=[" << gains.base_gains.Imax.transpose() << "]" << std::endl;
  os << "arm gains: " << std::endl;
  os << " - kp=[" << gains.arm_gains.Kp.transpose() << "]" << std::endl;
  os << " - kd=[" << gains.arm_gains.Kd.transpose() << "]" << std::endl;
  os << " - ki=[" << gains.arm_gains.Ki.transpose() << "]" << std::endl;
  os << " - i_max=[" << gains.arm_gains.Imax.transpose() << "]" << std::endl;
  os << "gripper gains: " << std::endl;
  os << " - kp=[" << gains.gripper_gains.Kp.transpose() << "]" << std::endl;
  os << " - kd=[" << gains.gripper_gains.Kd.transpose() << "]" << std::endl;
  os << " - ki=[" << gains.gripper_gains.Ki.transpose() << "]" << std::endl;
  os << " - i_max=[" << gains.gripper_gains.Imax.transpose() << "]" << std::endl;
  return os;
}