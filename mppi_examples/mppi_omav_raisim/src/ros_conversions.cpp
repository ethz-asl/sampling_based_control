//
// Created by studigem on 30.03.21.
//

#include "mppi_omav_raisim/ros_conversions.h"

namespace omav_raisim::conversions {
void to_thrust(const observation_t &x_nom, geometry_msgs::Vector3 &thrust) {
  thrust.x = x_nom(0);
  thrust.y = x_nom(1);
  thrust.z = x_nom(2);
}

void to_torque(const observation_t &x_nom, geometry_msgs::Vector3 &torque) {
  torque.x = x_nom(3);
  torque.y = x_nom(4);
  torque.z = x_nom(5);
}

void to_thrust_rate(const input_t &u, geometry_msgs::Vector3 &thrust_rate) {
  thrust_rate.x = u(0);
  thrust_rate.y = u(1);
  thrust_rate.z = u(2);
}

void to_torque_rate(const input_t &u, geometry_msgs::Vector3 &torque_rate) {
  torque_rate.x = u(3);
  torque_rate.y = u(4);
  torque_rate.z = u(5);
}
} // namespace omav_raisim::conversions
