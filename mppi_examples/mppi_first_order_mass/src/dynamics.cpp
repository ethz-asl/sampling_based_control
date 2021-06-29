/*!
 * @file     pendulum_cart_dynamics.cpp
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_first_order_mass/dynamics.h"

using namespace mppi;
namespace fom {

mppi::observation_t FOMDynamics::step(const mppi::input_t &u, const double dt) {
  x_ += u * dt_;
  return x_;
}

const mppi::observation_t FOMDynamics::get_state() const { return x_; }

void FOMDynamics::reset(const mppi::observation_t &x) { x_ = x; }
}  // namespace fom
