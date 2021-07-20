/*!
 * @file     pendulum_cart_dynamics.cpp
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_pathfinder/dynamics.h"

using namespace mppi;
namespace pathfinder {

DynamicsBase::observation_t PathfinderDynamics::step(
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

const DynamicsBase::observation_t PathfinderDynamics::get_state() const {
  return x_;
}

void PathfinderDynamics::reset(const DynamicsBase::observation_t &x) { x_ = x; }

void PathfinderDynamics::compute_velocities(double a) {
  xd_(0) = a - x_(0);
  xd_(1) = x_(0);
  xd_(2) = x_(1);
}

void PathfinderDynamics::integrate_internal(double u, double dt) {
  if (dt > config_.dt_internal) {
    std::stringstream ss;
    ss << "Integrate internal called with dt larger that internal dt: " << dt
       << "> " << config_.dt_internal;
    throw std::runtime_error(ss.str());
  }
  compute_velocities(u);
  x_ += xd_ * dt;
}
}  // namespace pathfinder
