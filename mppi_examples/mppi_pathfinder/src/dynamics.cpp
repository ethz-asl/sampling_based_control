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

void PathfinderDynamics::compute_velocities(double v_ref, double theta_ref) {
  xd_(0) = x_(3)*std::cos(x_(2));
  xd_(1) = x_(3)*std::sin(x_(2));
  xd_(2) = (theta_ref - x_(2))/config_.tau_theta;
  xd_(3) = (v_ref - x_(3))/0.5;
}

void PathfinderDynamics::integrate_internal(double v_ref, double theta_ref, double dt) {
  if (dt > config_.dt_internal) {
    std::stringstream ss;
    ss << "Integrate internal called with dt larger that internal dt: " << dt
       << "> " << config_.dt_internal;
    throw std::runtime_error(ss.str());
  }

  compute_velocities(v_ref, theta_ref);
  x_ += xd_ * dt;
}

DynamicsBase::observation_t PathfinderDynamics::step(
    const DynamicsBase::input_t &u, const double dt) {
  size_t steps = std::floor(dt / config_.dt_internal);
  if (steps > 0) {
    for (size_t i = 0; i < steps; i++)
      integrate_internal(u(0), u(1), config_.dt_internal);
  }
  double dt_last = dt - steps * config_.dt_internal;
  integrate_internal(u(0), u(1), dt_last);
  return x_;
}

const DynamicsBase::observation_t PathfinderDynamics::get_state() const {
  return x_;
}

void PathfinderDynamics::reset(const DynamicsBase::observation_t &x) { x_ = x; }
}  // namespace pathfinder
