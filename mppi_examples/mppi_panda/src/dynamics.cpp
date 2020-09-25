/*!
 * @file     pendulum_cart_dynamics.cpp
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_panda/dynamics.h"
#include <pinocchio/algorithm/rnea.hpp>

using namespace mppi;
namespace  panda{

DynamicsBase::observation_t PandaDynamics::step(const DynamicsBase::input_t &u, const double dt) {

  double dt_internal = dt / config_.substeps;

  if (kinematic_simulation_){
    for(size_t i=0; i<config_.substeps; i++)
      x_.head<7>() += u * dt_internal;
  }
  else{
    constexpr double viscous_joint_friction = 0.5;
    for(size_t i=0; i<config_.substeps; i++){
      xdd_ = pinocchio::aba(model_, data_, x_.head<7>(), x_.tail<7>(), u - viscous_joint_friction * x_.tail<7>());
      x_.tail<7>() += xdd_ * dt_internal;
      x_.head<7>() += x_.tail<7>() * dt_internal;
    }
  }
  return x_;
}

void PandaDynamics::reset(const DynamicsBase::observation_t &x) { x_ = x; }

DynamicsBase::input_t PandaDynamics::get_zero_input(const observation_t& x) {
  if (kinematic_simulation_)
    return DynamicsBase::input_t::Zero(get_input_dimension());
  else{
    return pinocchio::computeGeneralizedGravity(model_, data_, x.head<7>());
  }
}

}
