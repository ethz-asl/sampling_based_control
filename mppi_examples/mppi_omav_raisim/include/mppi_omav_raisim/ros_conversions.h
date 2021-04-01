//
// Created by studigem on 30.03.21.
//

#pragma once

#include "mppi/dynamics/dynamics_base.h"
#include <geometry_msgs/Vector3.h>

namespace omav_raisim::conversions {
using observation_t = Eigen::VectorXd;
using input_t = Eigen::VectorXd;
void to_thrust(const observation_t &x_nom, geometry_msgs::Vector3 &thrust);
void to_torque(const observation_t &x_nom, geometry_msgs::Vector3 &torque);
void to_thrust_rate(const input_t &u, geometry_msgs::Vector3 &thrust_rate);
void to_torque_rate(const input_t &u, geometry_msgs::Vector3 &torque_rate);

}  // namespace omav_raisim::conversions
