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
  double dt_internal = dt / config_.substeps;
  for (size_t i = 0; i < config_.substeps; i++) {
    x_.head<7>() += u * dt_internal;
  }
  return x_;
}

void PandaDynamics::reset(const DynamicsBase::observation_t &x) { x_ = x; }

DynamicsBase::input_t PandaDynamics::get_zero_input(const observation_t &x) {
  return DynamicsBase::input_t::Zero(get_input_dimension());
}

}  // namespace panda
