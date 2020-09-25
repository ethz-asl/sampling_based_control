/*!
 * @file     pendulum_cart_dynamics.cpp
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_panda_mobile/dynamics.h"

using namespace mppi;
namespace  panda_mobile{

DynamicsBase::observation_t PandaMobileDynamics::step(const DynamicsBase::input_t &u, const double dt) {
  x_ += u * dt;
  return x_;
}

void PandaMobileDynamics::reset(const DynamicsBase::observation_t &x) { x_ = x; }

DynamicsBase::input_t PandaMobileDynamics::get_zero_input(const observation_t& x) {
    return DynamicsBase::input_t::Zero(get_input_dimension());
}

}
