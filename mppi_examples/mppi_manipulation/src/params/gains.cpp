//
// Created by giuseppe on 09.08.21.
//

#include "mppi_manipulation/params/gains.h"

using namespace manipulation;

bool PDGains::init_from_ros(const ros::NodeHandle& nh) {
  if (!base_gains.parse_from_ros(nh, "dynamics/gains/base_gains")) return false;
  if (!arm_gains.parse_from_ros(nh, "dynamics/gains/arm_gains")) return false;
  if (!gripper_gains.parse_from_ros(nh, "dynamics/gains/gripper_gains")) return false;
  return true;
}
