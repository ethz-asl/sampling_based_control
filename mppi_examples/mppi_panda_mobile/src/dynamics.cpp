/*!
 * @file     pendulum_cart_dynamics.cpp
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_panda_mobile/dynamics.h"

using namespace mppi;
namespace panda_mobile {

PandaMobileDynamics::PandaMobileDynamics(const std::string& robot_description,
                                         bool holonomic)
    : robot_description_(robot_description), holonomic_(holonomic) {
  // init model
  x_ = observation_t::Zero(PandaMobileDim::STATE_DIMENSION);
}

mppi::observation_t PandaMobileDynamics::step(const mppi::input_t& u,
                                              const double dt) {
  // integrate joint velocities
  x_.tail<7>() += u.tail<7>() * dt;

  // base velocity in in body frame
  double& yaw = x_(2);
  const double& vx = u(0);
  const double& vy = u(1);
  const double& yawd = u(2);

  if (holonomic_) {
    x_(0) += (vx * std::cos(yaw) - vy * std::sin(yaw)) * dt;
    x_(1) += (vx * std::sin(yaw) + vy * std::cos(yaw)) * dt;
  } else {
    x_(0) += vx * std::cos(yaw) * dt;
    x_(1) += vx * std::sin(yaw) * dt;
  }
  x_(2) += yawd * dt;
  return x_;
}

const mppi::observation_t PandaMobileDynamics::get_state() const { return x_; }

void PandaMobileDynamics::reset(const mppi::observation_t& x) { x_ = x; }

mppi::input_t PandaMobileDynamics::get_zero_input(const observation_t& x) {
  return mppi::input_t::Zero(get_input_dimension());
}

}  // namespace panda_mobile
