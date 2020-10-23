/*!
 * @file     dynamics_raisim.cpp
 * @author   Giuseppe Rizzi
 * @date     05.10.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_panda_raisim/dynamics.h"
#include <ros/package.h>

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

  /// create raisim objects
  std::string dir = ros::package::getPath("mppi_panda_raisim");;
  std::string door_urdf_path = dir + "/data/door.urdf";
  door = sim_.addArticulatedSystem(door_urdf_path);
  door->setGeneralizedForce(Eigen::VectorXd::Zero(door->getDOF()));
  door->setGeneralizedCoordinate(Eigen::VectorXd::Zero(1));
  door->setBaseOrientation(Eigen::Matrix3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())));

}

void PandaRaisimDynamics::initialize_pd() {
  ///panda
  cmd.setZero(PandaDim::JOINT_DIMENSION);
  cmdv.setZero(PandaDim::JOINT_DIMENSION);
  joint_p.setZero(PandaDim::JOINT_DIMENSION);
  joint_v.setZero(PandaDim::JOINT_DIMENSION);
  joint_p_gain.setZero(PandaDim::JOINT_DIMENSION);
  joint_d_gain.setZero(PandaDim::JOINT_DIMENSION);
  joint_p_desired.setZero(PandaDim::JOINT_DIMENSION);
  joint_v_desired.setZero(PandaDim::JOINT_DIMENSION);
  joint_p_gain.head(PandaDim::ARM_DIMENSION).setConstant(0);
  joint_d_gain.head(PandaDim::ARM_DIMENSION).setConstant(10.0);
  joint_p_gain.tail(PandaDim::GRIPPER_DIMENSION).setConstant(0);
  joint_d_gain.tail(PandaDim::GRIPPER_DIMENSION).setConstant(10.0);
  panda->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  panda->setPdGains(joint_p_gain, joint_d_gain);

  /// door
  //door->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  //door->setPdGains(Eigen::VectorXd::Ones(1)*20, Eigen::VectorXd::Ones(1)*1.0);
  //door->setPdTarget(Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1));
  //door->setName("door");

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
  // no mimic support --> 1 input for gripper but 2 joints to control
  cmdv.head<PandaDim::INPUT_DIMENSION>() = u;
  cmdv(PandaDim::JOINT_DIMENSION-1) = u(PandaDim::INPUT_DIMENSION-1);
  panda->setPdTarget(cmd, cmdv);
  panda->setGeneralizedForce(panda->getNonlinearities());
  panda->getState(joint_p, joint_v);

  // gravity compensated door
  door->getState(door_p, door_v);
  door->setGeneralizedForce(door->getNonlinearities());

  // step simulation
  sim_.integrate();

  x_.head<PandaDim::JOINT_DIMENSION>() = joint_p;
  x_.segment<PandaDim::JOINT_DIMENSION>(PandaDim::JOINT_DIMENSION) = joint_v;
  x_.segment<2*PandaDim::DOOR_DIMENSION>(2*PandaDim::JOINT_DIMENSION)(0) = door_p(0);
  x_.segment<2*PandaDim::DOOR_DIMENSION>(2*PandaDim::JOINT_DIMENSION)(1) = door_v(0);

  return x_;
}

void PandaRaisimDynamics::reset(const DynamicsBase::observation_t &x) {
  // internal eigen state
  x_ = x;

  // reset arm
  panda->setState(x_.head<PandaDim::JOINT_DIMENSION>(),
                  x_.segment<PandaDim::JOINT_DIMENSION>(PandaDim::JOINT_DIMENSION));
  panda->setGeneralizedForce(Eigen::VectorXd::Zero(panda->getDOF()));

  // reset door
  door->setState(x_.segment<PandaDim::DOOR_DIMENSION>(2*PandaDim::JOINT_DIMENSION),
                 x_.segment<PandaDim::DOOR_DIMENSION>(2*PandaDim::JOINT_DIMENSION+1));
  door->setGeneralizedForce(Eigen::VectorXd::Zero(door->getDOF()));
}

DynamicsBase::input_t PandaRaisimDynamics::get_zero_input(const observation_t& x) {
  return DynamicsBase::input_t::Zero(get_input_dimension());
}

}
