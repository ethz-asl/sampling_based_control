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

PandaRaisimDynamics::PandaRaisimDynamics(const std::string& robot_description, const double dt){
  initialize_world(robot_description, dt);
  initialize_pd();
  set_collision();
};

void PandaRaisimDynamics::initialize_world(const std::string robot_description, const double dt) {
  dt_ = dt;
  x_ = observation_t::Zero(PandaDim::STATE_DIMENSION);
  sim_.setTimeStep(dt);
  sim_.setERP(0.,0.);
  robot_description_ = robot_description;
  panda = sim_.addArticulatedSystem(robot_description_, "/");
  panda->setGeneralizedForce(Eigen::VectorXd::Zero(panda->getDOF()));
}

void PandaRaisimDynamics::initialize_pd() {
  joint_p.setZero(PandaDim::JOINT_DIMENSION);
  joint_v.setZero(PandaDim::JOINT_DIMENSION);
  joint_p_gain.setZero(PandaDim::JOINT_DIMENSION);
  joint_d_gain.setZero(PandaDim::JOINT_DIMENSION);
  joint_p_desired.setZero(PandaDim::JOINT_DIMENSION);
  joint_v_desired.setZero(PandaDim::JOINT_DIMENSION);
  joint_p_gain.head(PandaDim::ARM_DIMENSION).setConstant(200);
  joint_d_gain.head(PandaDim::ARM_DIMENSION).setConstant(10.0);
  joint_p_gain.tail(PandaDim::GRIPPER_DIMENSION).setConstant(200);
  joint_d_gain.tail(PandaDim::GRIPPER_DIMENSION).setConstant(10.0);
  panda->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  panda->setPdGains(joint_p_gain, joint_d_gain);
}

void PandaRaisimDynamics::set_collision() {
  std::vector<int> pandaBodyIdxs;
  for (const auto& bodyName : panda->getBodyNames())
    pandaBodyIdxs.push_back(panda->getBodyIdx(bodyName));
  for (const auto body_idx1 : pandaBodyIdxs)
    for (const auto body_idx2 : pandaBodyIdxs)
      panda->ignoreCollisionBetween(body_idx1, body_idx2);
}

DynamicsBase::observation_t PandaRaisimDynamics::step(const DynamicsBase::input_t &u, const double dt) {
  // integrate desired velocity
  // no mimic support --> 1 input for gripper but 2 joints to control
  x_.tail<PandaDim::JOINT_DIMENSION>().head<PandaDim::INPUT_DIMENSION>() += u*dt;
  x_.tail<1>() += u.tail<1>() * dt;

  // set pd target
  panda->setPdTarget(x_.tail<PandaDim::JOINT_DIMENSION>(), Eigen::VectorXd::Zero(PandaDim::JOINT_DIMENSION));
  panda->getState(joint_p, joint_v);

  // step simulation
  sim_.integrate();

  x_.head<PandaDim::JOINT_DIMENSION>() = joint_p;
  x_.segment<PandaDim::JOINT_DIMENSION>(PandaDim::JOINT_DIMENSION) = joint_v;
  return x_;
}

void PandaRaisimDynamics::reset(const DynamicsBase::observation_t &x) {
  x_ = x;
  panda->setState(x_.head<PandaDim::JOINT_DIMENSION>(),
                  x_.segment<PandaDim::JOINT_DIMENSION>(PandaDim::JOINT_DIMENSION));
  panda->setGeneralizedForce(Eigen::VectorXd::Zero(panda->getDOF()));
}

DynamicsBase::input_t PandaRaisimDynamics::get_zero_input(const observation_t& x) {
  return DynamicsBase::input_t::Zero(get_input_dimension());
}

}
