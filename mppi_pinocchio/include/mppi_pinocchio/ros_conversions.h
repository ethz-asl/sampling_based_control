//
// Created by giuseppe on 01.03.21.
//

#pragma once

// clang-format off
#include "mppi_pinocchio/model.h"
#include <geometry_msgs/Pose.h>
// clang-format on

namespace mppi_pinocchio {

void to_msg(const Pose& pose, geometry_msgs::Pose& pose_ros);
}