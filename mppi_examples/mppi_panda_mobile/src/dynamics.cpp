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

void PandaMobileDynamics::step(observation_t& x, const DynamicsBase::input_t &u, const double dt) {
  // integrate joint velocities
  x_.head<7>() += u.head<7>() * dt;
  
  // base velocity in in body frame
  double& yaw = x_(9);
  const double& vx = u(7);
  const double& vy = u(8);
  const double& yawd = u(9);

  x_(7) += ( vx * std::cos(yaw) + vy * std::sin(yaw)) * dt;
  x_(8) += (-vx * std::sin(yaw) + vy * std::cos(yaw)) * dt;
  x_(9) += yawd * dt;
  x = x_;
}

const DynamicsBase::observation_t PandaMobileDynamics::get_state() const { return x_; }

void PandaMobileDynamics::reset(const DynamicsBase::observation_t &x) { x_ = x; }

DynamicsBase::input_t PandaMobileDynamics::get_zero_input(const observation_t& x) {
    return DynamicsBase::input_t::Zero(get_input_dimension());
}

}
