/*!
 * @file     dynamics_raisim.cpp
 * @author   Giuseppe Rizzi
 * @date     05.10.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_manipulation/dynamics.h"
#include <ros/package.h>

namespace manipulation {

PandaRaisimDynamics::PandaRaisimDynamics(const DynamicsParams& params)
    : params_(params) {
  initialize_world(params_.robot_description, params_.object_description);
  initialize_pd();
  set_collision();
  if (params_.has_filter) {
    sf_ = std::make_unique<PandaMobileSafetyFilter>(params_.filter_params);
  } else {
    sf_ = nullptr;
  }
  t_ = 0.0;
  ee_force_applied_ = false;
};

void PandaRaisimDynamics::initialize_world(
    const std::string& robot_description,
    const std::string& object_description) {
  dt_ = params_.dt;
  sim_.setTimeStep(params_.dt);
  sim_.setERP(0., 0.);

  gravity_.e() << 0.0, 0.0, -9.81;
  sim_.setGravity(gravity_);

  sim_.setMaterialPairProp("steel", "steel", 0.1, 0.0, 0.0);
  robot_description_ = robot_description;
  panda = sim_.addArticulatedSystem(robot_description_, "/");

  tau_ext_ = Eigen::VectorXd::Zero(panda->getDOF());
  J_contact_.setZero(3, panda->getDOF());

  /// create raisim objects
  object_description_ = object_description;
  object = sim_.addArticulatedSystem(object_description_, "/");

  // robot dof
  robot_dof_ = BASE_ARM_GRIPPER_DIM;
  state_dimension_ = STATE_DIMENSION;
  input_dimension_ = INPUT_DIMENSION;
  x_ = mppi::observation_t::Zero(STATE_DIMENSION);

  reset(params_.initial_state, t_);
}

void PandaRaisimDynamics::initialize_pd() {
  /// panda
  cmd.setZero(robot_dof_);
  cmdv.setZero(robot_dof_);
  joint_p.setZero(robot_dof_);
  joint_v.setZero(robot_dof_);
  joint_p_gain.setZero(robot_dof_);
  joint_d_gain.setZero(robot_dof_);
  joint_p_desired.setZero(robot_dof_);
  joint_v_desired.setZero(robot_dof_);

  // clang-format off
  joint_p_gain.head(BASE_DIMENSION) = params_.gains.base_gains.Kp;
  joint_d_gain.head(BASE_DIMENSION) = params_.gains.base_gains.Kd;
  joint_p_gain.segment(BASE_DIMENSION, ARM_DIMENSION) =
      params_.gains.arm_gains.Kp;
  joint_d_gain.segment(BASE_DIMENSION, ARM_DIMENSION) =
      params_.gains.arm_gains.Kd;
  joint_p_gain.tail(GRIPPER_DIMENSION) = params_.gains.gripper_gains.Kp;
  joint_d_gain.tail(GRIPPER_DIMENSION) = params_.gains.gripper_gains.Kd;
  // clang-format on

  panda->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  panda->setPdGains(joint_p_gain, joint_d_gain);
  panda->setGeneralizedForce(Eigen::VectorXd::Zero(panda->getDOF()));

  object->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  object->setPdGains(Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1));
  object->setGeneralizedForce({0.0});
}

void PandaRaisimDynamics::set_collision() {
  std::vector<int> pandaBodyIdxs;
  for (const auto& bodyName : panda->getBodyNames())
    pandaBodyIdxs.push_back(panda->getBodyIdx(bodyName));
  for (const auto body_idx1 : pandaBodyIdxs)
    for (const auto body_idx2 : pandaBodyIdxs)
      panda->ignoreCollisionBetween(body_idx1, body_idx2);
}

void PandaRaisimDynamics::set_control(const mppi::input_t& u) {
  // safety filter (optional)
  if (sf_) {
    sf_->update(x_, u, t_);
    if (params_.apply_filter) {
      sf_->apply(u_opt_);
    } else {
      u_opt_ = u.head<10>();
    }
  } else {
    u_opt_ = u;
  }

  // keep the gripper in the current position
  cmd.tail<PandaDim::GRIPPER_DIMENSION>()
      << x_.head<BASE_ARM_GRIPPER_DIM>().tail<GRIPPER_DIMENSION>();

  cmdv(0) = u_opt_(0) * std::cos(x_(2)) - u_opt_(1) * std::sin(x_(2));
  cmdv(1) = u_opt_(0) * std::sin(x_(2)) + u_opt_(1) * std::cos(x_(2));
  cmdv(2) = u_opt_(2);
  cmdv.segment<ARM_DIMENSION>(BASE_DIMENSION) =
      u_opt_.segment<ARM_DIMENSION>(BASE_DIMENSION);

  cmdv.tail<PandaDim::GRIPPER_DIMENSION>().setZero();
  panda->setPdTarget(cmd, cmdv);
  panda->setGeneralizedForce(panda->getNonlinearities(gravity_));

  // gravity compensated object
  object->setGeneralizedForce(object->getNonlinearities(gravity_));
}

void PandaRaisimDynamics::advance() {
  // get contact state
  double in_contact = -1;
  for (const auto& contact : object->getContacts()) {
    if (!contact.skip() && !contact.isSelfCollision()) {
      in_contact = 1;
      break;
    }
  }

  // integrate the tank
  get_external_torque(tau_ext_);
  tank_.step(u_opt_.transpose()*tau_ext_, sim_.getTimeStep());

  // step simulation
  sim_.integrate();
  t_ += sim_.getTimeStep();

  panda->getState(joint_p, joint_v);
  object->getState(object_p, object_v);

  x_.head<BASE_ARM_GRIPPER_DIM>() = joint_p;
  x_.segment<BASE_ARM_GRIPPER_DIM>(BASE_ARM_GRIPPER_DIM) = joint_v;
  x_.segment<2 * OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM)(0) = object_p(0);
  x_.segment<2 * OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM)(1) = object_v(0);
  x_(2 * BASE_ARM_GRIPPER_DIM + 2 * OBJECT_DIMENSION) = in_contact;
  x_(2 * BASE_ARM_GRIPPER_DIM + 2 * OBJECT_DIMENSION + 1) = tank_.get_state();
  x_.tail<TORQUE_DIMENSION>() = tau_ext_;
}

mppi::observation_t PandaRaisimDynamics::step(const mppi::input_t& u,
                                              const double dt) {
  set_control(u);
  advance();
  return x_;
}

void PandaRaisimDynamics::reset(const mppi::observation_t& x, const double t) {
  // internal eigen state
  t_ = t;
  x_ = x;

  // reset arm
  panda->setState(x_.head<BASE_ARM_GRIPPER_DIM>(),
                  x_.segment<BASE_ARM_GRIPPER_DIM>(BASE_ARM_GRIPPER_DIM));
  object->setState(x_.segment<OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM),
                   x_.segment<OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM + 1));
  tank_.reset(x(2 * BASE_ARM_GRIPPER_DIM + 2 * OBJECT_DIMENSION + 1), t);
  if (sf_) {
    sf_->reset_constraints();
  }
}

mppi::input_t PandaRaisimDynamics::get_zero_input(
    const mppi::observation_t& x) {
  return mppi::input_t::Zero(get_input_dimension());
}

void PandaRaisimDynamics::get_end_effector_pose(
    Eigen::Vector3d& position, Eigen::Quaterniond& orientation) {
  size_t frame_id = panda->getFrameIdxByName("panda_grasp_joint");
  raisim::Vec<3> pos;
  raisim::Mat<3, 3> rot;
  panda->getFramePosition(frame_id, pos);
  panda->getFrameOrientation(frame_id, rot);
  position = pos.e();
  orientation = Eigen::Quaterniond(rot.e());
}

void PandaRaisimDynamics::get_handle_pose(Eigen::Vector3d& position,
                                          Eigen::Quaterniond& orientation) {
  size_t frame_id = object->getFrameIdxByName(params_.object_handle_joint);
  raisim::Vec<3> pos;
  raisim::Mat<3, 3> rot;
  object->getFramePosition(frame_id, pos);
  object->getFrameOrientation(frame_id, rot);
  position = pos.e();
  orientation = Eigen::Quaterniond(rot.e());
}

std::vector<force_t> PandaRaisimDynamics::get_contact_forces() {
  std::vector<force_t> forces;
  for (const auto contact : panda->getContacts()) {
    if (contact.skip())
      continue;  /// if the contact is internal, one contact point is set to
                 /// 'skip'
    if (contact.isSelfCollision()) continue;
    force_t force;
    force.force = -contact.getContactFrame().e().transpose() * contact.getImpulse().e() /
                  sim_.getTimeStep();
    force.position = contact.getPosition().e();
    forces.push_back(force);
  }
  return forces;
}

void PandaRaisimDynamics::get_external_torque(Eigen::VectorXd& tau) {
  tau.setZero((int)panda->getDOF());
  for (const auto contact : panda->getContacts()) {
    J_contact_.setZero();
    if (!contact.skip() && !contact.isSelfCollision()) {
      panda->getDenseJacobian(contact.getlocalBodyIndex(),
                              contact.getPosition(), J_contact_);

      // clang-format off
      // the base jacobian references the world frame, need to rotate
      // to the base reference frame
      J_contact_.topLeftCorner<3, 3>() << std::cos(x_(2)), std::sin(x_(2)), 0,
                                          -std::sin(x_(2)), std::cos(x_(2)), 0,
                                          0, 0, 1;
      // transform contact to force --> to force in world frame --> to reaction torques
      tau -= J_contact_.transpose() * contact.getContactFrame().e().transpose() * contact.getImpulse().e() / sim_.getTimeStep();
      // clang-format on
    }
  }
  if (ee_force_applied_){
    J_contact_.setZero();
    panda->getDenseFrameJacobian("panda_grasp_joint", J_contact_);
    J_contact_.topLeftCorner<3, 3>() << std::cos(x_(2)), -std::sin(x_(2)), 0,
    std::sin(x_(2)), std::cos(x_(2)), 0,
    0, 0, 1;
    tau += J_contact_.transpose() * panda->getExternalForce()[0].e();

  }
}

void PandaRaisimDynamics::get_external_wrench(Eigen::VectorXd& wrench) {
  wrench.setZero(6);
  size_t frame_id = panda->getFrameIdxByName("panda_grasp_joint");
  raisim::Vec<3> pos;
  raisim::Mat<3, 3> rot;
  panda->getFramePosition(frame_id, pos);
  panda->getFrameOrientation(frame_id, rot);

  for (const auto contact : panda->getContacts()) {
    if (!contact.skip() && !contact.isSelfCollision()) {
      // ee_frame <-- world_frame <-- force <-- impulse
      Eigen::Vector3d force_ee_frame =
          -rot.e().transpose() * contact.getContactFrame().e().transpose() *
          contact.getImpulse().e() / sim_.getTimeStep();
      Eigen::Vector3d relative_position =
          rot.e().transpose() * (contact.getPosition().e() - pos.e());
      wrench.head<3>() += force_ee_frame;
      wrench.tail<3>() = relative_position.cross(force_ee_frame);
    }
  }
  if (ee_force_applied_) {
    wrench.head<3>() += panda->getExternalForce()[0].e();
  }
}

void PandaRaisimDynamics::get_reference_link_pose(Eigen::Vector3d& position,
                             Eigen::Quaterniond& orientation){
  size_t frame_id = panda->getFrameIdxByName("reference_link_joint");
  raisim::Vec<3> pos;
  raisim::Mat<3, 3> rot;
  panda->getFramePosition(frame_id, pos);
  panda->getFrameOrientation(frame_id, rot);
  position = pos.e();
  orientation = Eigen::Quaterniond(rot.e());
}

void PandaRaisimDynamics::get_ee_jacobian(Eigen::MatrixXd& J){
  J.setZero(6, (int)panda->getDOF());
  Eigen::MatrixXd J_linear;
  J_linear.setZero(3, 12);
  Eigen::MatrixXd J_angular;
  J_angular.setZero(3, 12);

  panda->getDenseFrameJacobian("panda_grasp_joint", J_linear);
  panda->getDenseFrameRotationalJacobian("panda_grasp_joint", J_angular);
  J.topRows(3) = J_linear;
  J.bottomRows(3) = J_angular;
  // clang-format off
  J.topLeftCorner<3, 3>() << std::cos(x_(2)), -std::sin(x_(2)), 0,
                             std::sin(x_(2)), std::cos(x_(2)), 0,
                             0, 0, 1;
  // clang-format on
}

void PandaRaisimDynamics::set_external_ee_force(const Eigen::Vector3d& f) {
  ee_force_applied_ = (f.norm() > 1e-4);
  auto& frame = panda->getFrameByName("panda_grasp_joint");
  panda->setExternalForce(frame.parentId, raisim::ArticulatedSystem::Frame::WORLD_FRAME, f, raisim::ArticulatedSystem::Frame::BODY_FRAME, raisim::Vec<3>());
}

double PandaRaisimDynamics::get_object_displacement() const {
  return x_.segment<OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM)(0);
}

void PandaRaisimDynamics::fix_object() {
  object->getState(object_p, object_v);
  std::vector<raisim::Vec<2>> object_limits;
  raisim::Vec<2> limit;
  limit[0] = object_p[0] - 0.001;
  limit[1] = object_p[0] + 0.001;
  object_limits.push_back(limit);
  object->setJointLimits(object_limits);
}

void PandaRaisimDynamics::release_object() {
  std::vector<raisim::Vec<2>> object_limits;
  raisim::Vec<2> limit;
  limit[0] = 0.0;
  limit[1] = M_PI_2;
  object_limits.push_back(limit);
  object->setJointLimits(object_limits);
}

}  // namespace manipulation
