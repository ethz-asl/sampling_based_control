/*!
 * @file     pendulum_cart_dynamics.cpp
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_panda/dynamics.h"

using namespace mppi;
namespace panda {

DynamicsBase::observation_t PandaDynamics::step(const DynamicsBase::input_t &u,
                                                const double dt) {
  x_.head<7>() += u * dt;
  return x_;
}

const DynamicsBase::observation_t PandaDynamics::get_state() const {
  return x_;
}

void PandaDynamics::reset(const DynamicsBase::observation_t &x) { x_ = x; }

DynamicsBase::input_t PandaDynamics::get_zero_input(const observation_t &x) {
  return DynamicsBase::input_t::Zero(get_input_dimension());
}

}  // namespace panda
