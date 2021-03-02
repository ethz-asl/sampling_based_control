/*!
 * @file     pendulum_cart_dynamics.cpp
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_pole_cart/dynamics.h"

using namespace mppi;
namespace pole_cart {

void PoleCartDynamics::compute_velocities(double F) {
  xd_(0) = x_(2);
  xd_(1) = x_(3);
  xd_(2) = (F - config_.mux * x_(2) +
            config_.mp * 9.81 * std::cos(x_(1)) * std::sin(x_(1)) +
            config_.mp * x_(1) * x_(1) * config_.l * std::sin(x_(1))) /
           (config_.mc + config_.mp * std::sin(x_(1)) * std::sin(x_(1)));
  xd_(3) = (-config_.mutheta * x_(3) - 9.81 * std::sin(x_(1)) -
            xd_(2) * std::cos(x_(1))) /
           config_.l;
}

void PoleCartDynamics::integrate_internal(double u, double dt) {
  if (dt > config_.dt_internal) {
    std::stringstream ss;
    ss << "Integrate internal called with dt larger that internal dt: " << dt
       << "> " << config_.dt_internal;
    throw std::runtime_error(ss.str());
  }

  compute_velocities(u);
  x_ += xd_ * dt;
}

DynamicsBase::observation_t PoleCartDynamics::step(
    const DynamicsBase::input_t &u, const double dt) {
  size_t steps = std::floor(dt / config_.dt_internal);
  if (steps > 0) {
    for (size_t i = 0; i < steps; i++)
      integrate_internal(u(0), config_.dt_internal);
  }
  double dt_last = dt - steps * config_.dt_internal;
  integrate_internal(u(0), dt_last);
  return x_;
}

const DynamicsBase::observation_t PoleCartDynamics::get_state() const {
  return x_;
}

void PoleCartDynamics::reset(const DynamicsBase::observation_t &x) { x_ = x; }
}  // namespace pole_cart
