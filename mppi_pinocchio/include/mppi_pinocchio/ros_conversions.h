//
// Created by giuseppe on 01.03.21.
//

#pragma once
#include "mppi_pinocchio/model.h"
#include <geometry_msgs/Pose.h>

namespace mppi_pinocchio{

void to_msg(const Pose& pose, geometry_msgs::Pose& pose_ros);
}