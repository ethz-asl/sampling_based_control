/*!
 * @file     dynamics_raisim.cpp
 * @author   Giuseppe Rizzi
 * @date     05.10.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_panda/dynamics_raisim.h"

using namespace std::chrono;
using namespace mppi;
namespace  panda{

DynamicsBase::observation_t PandaRaisimDynamics::step(const DynamicsBase::input_t &u, const double dt) {

//  x_cmd_.head(7) += u*dt;
  //panda->setPdTarget(x_cmd_.head(7) + u*dt, Eigen::VectorXd::Zero(7));
  panda->setPdTarget(u, Eigen::VectorXd::Zero(7));
  panda->getState(joint_p, joint_v);

  auto start = steady_clock::now();
  sim_.integrate();
  auto end = steady_clock::now();

  double delta = duration_cast<nanoseconds>(end-start).count() / 1e6;
  time_recordings_.push_back(delta);
  x_.head<7>() = joint_p;
  x_.tail<7>() = joint_v;
  std::cout << "Contact problem size is: " << sim_.getContactProblem()->size() << std::endl;
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
  // control in joint position: zero joint position is staying at the same place
  return x.head(7);
  //return DynamicsBase::input_t::Zero(get_input_dimension());
}

}
