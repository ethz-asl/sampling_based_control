/*!
 * @file     dynamics_raisim.cpp
 * @author   Giuseppe Rizzi
 * @date     05.10.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_panda_raisim/dynamics.h"

using namespace std::chrono;
using namespace mppi;
namespace  panda{

DynamicsBase::observation_t PandaRaisimDynamics::step(const DynamicsBase::input_t &u, const double dt) {

  x_.tail<7>() += u*dt;
  panda->setPdTarget(x_.tail<PandaDim::JOINT_DIMENSION>(), Eigen::VectorXd::Zero(PandaDim::JOINT_DIMENSION));
  panda->getState(joint_p, joint_v);
  sim_.integrate();

  x_.head<PandaDim::JOINT_DIMENSION>() = joint_p;
  x_.segment<PandaDim::JOINT_DIMENSION>(PandaDim::JOINT_DIMENSION) = joint_v;
  return x_;
}

void PandaRaisimDynamics::reset(const DynamicsBase::observation_t &x) {
  x_ = x;
  panda->setState(x_.head<PandaDim::JOINT_DIMENSION>(),
                  x_.segment<PandaDim::JOINT_DIMENSION>(PandaDim::JOINT_DIMENSION));
  panda->setPdTarget(x_.head<PandaDim::JOINT_DIMENSION>(),
                     Eigen::VectorXd::Zero(PandaDim::JOINT_DIMENSION));
  panda->setGeneralizedForce(Eigen::VectorXd::Zero(panda->getDOF()));
}

DynamicsBase::input_t PandaRaisimDynamics::get_zero_input(const observation_t& x) {
  return DynamicsBase::input_t::Zero(get_input_dimension());
}

}
