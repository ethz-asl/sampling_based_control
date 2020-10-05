/*!
 * @file     dynamics_raisim.cpp
 * @author   Giuseppe Rizzi
 * @date     05.10.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_panda/dynamics_raisim.h"

using namespace mppi;
namespace  panda{

DynamicsBase::observation_t PandaRaisimDynamics::step(const DynamicsBase::input_t &u, const double dt) {

  x_cmd_.head(7) += u*dt;
  panda->setPdTarget(x_cmd_.head(7), u);
  panda->getState(joint_p, joint_v);
  sim_.integrate();
  x_.head<7>() = joint_p;
  x_.tail<7>() = joint_v;
  return x_;
}

void PandaRaisimDynamics::reset(const DynamicsBase::observation_t &x) {
  x_ = x;
  x_cmd_ = x;
  panda->setState(x_.head<7>(), x_.tail<7>());
  panda->setPdTarget(x_.head<7>(), x_.tail<7>());
  panda->setGeneralizedForce(Eigen::VectorXd::Zero(panda->getDOF()));
}

DynamicsBase::input_t PandaRaisimDynamics::get_zero_input(const observation_t& x) {
  return DynamicsBase::input_t::Zero(get_input_dimension());
}

}
