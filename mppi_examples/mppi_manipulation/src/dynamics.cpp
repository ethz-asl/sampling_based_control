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
};

void PandaRaisimDynamics::initialize_world(
    const std::string& robot_description,
    const std::string& object_description) {
  dt_ = params_.dt;
  sim_.setTimeStep(params_.dt);
  sim_.setERP(0., 0.);
  sim_.setMaterialPairProp("steel", "steel", 0.01, 0.0, 0.0);
  robot_description_ = robot_description;
  panda = sim_.addArticulatedSystem(robot_description_, "/");
  panda->setGeneralizedForce(Eigen::VectorXd::Zero(panda->getDOF()));
  tau_ext_ = Eigen::VectorXd::Zero(panda->getDOF());

  /// create raisim objects
  object_description_ = object_description;
  object = sim_.addArticulatedSystem(object_description_, "/");
  object->setGeneralizedForce(Eigen::VectorXd::Zero(object->getDOF()));
  object->setGeneralizedCoordinate(Eigen::VectorXd::Zero(1));

  // robot dof
  robot_dof_ = BASE_ARM_GRIPPER_DIM;
  state_dimension_ =
      2 * (BASE_ARM_GRIPPER_DIM + OBJECT_DIMENSION) + CONTACT_STATE;
  input_dimension_ = BASE_ARM_GRIPPER_DIM - 1;  // mimic joint for gripper
  x_ = mppi::observation_t::Zero(state_dimension_);

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
}

void PandaRaisimDynamics::set_collision() {
  std::vector<int> pandaBodyIdxs;
  for (const auto& bodyName : panda->getBodyNames())
    pandaBodyIdxs.push_back(panda->getBodyIdx(bodyName));
  for (const auto body_idx1 : pandaBodyIdxs)
    for (const auto body_idx2 : pandaBodyIdxs)
      panda->ignoreCollisionBetween(body_idx1, body_idx2);
}

mppi::observation_t PandaRaisimDynamics::step(const mppi::input_t& u,
                                              const double dt) {
  // safety filter (optional)
  if (sf_) {
    get_external_torque(torque_ext_);
    sf_->update(x_, u, torque_ext_, t_);
    if (params_.apply_filter) {
      sf_->apply(u_opt_);
    } else {
      u_opt_ = u.head<10>();
    }
    sf_->passivity_constraint()->integrate_tank(u_opt_);
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
  panda->setGeneralizedForce(panda->getNonlinearities());
  panda->getState(joint_p, joint_v);

  // gravity compensated object
  object->getState(object_p, object_v);
  object->setGeneralizedForce(object->getNonlinearities());

  // get contact state
  double in_contact = -1;
  for (const auto& contact : object->getContacts()) {
    if (!contact.skip() && !contact.isSelfCollision()) {
      in_contact = 1;
      break;
    }
  }

  pre_integrate();

  // step simulation
  sim_.integrate();
  t_ += sim_.getTimeStep();

  x_.head<BASE_ARM_GRIPPER_DIM>() = joint_p;
  x_.segment<BASE_ARM_GRIPPER_DIM>(BASE_ARM_GRIPPER_DIM) = joint_v;
  x_.segment<2 * OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM)(0) = object_p(0);
  x_.segment<2 * OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM)(1) = object_v(0);
  x_.tail<1>()(0) = in_contact;
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
  if (sf_) sf_->reset_constraints();
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
  size_t frame_id = object->getFrameIdxByName("handle_joint");
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
    force.force = -contact.getContactFrame().e() * contact.getImpulse()->e() /
                  sim_.getTimeStep();
    force.position = contact.getPosition().e();
    forces.push_back(force);
  }
  return forces;
}

void PandaRaisimDynamics::get_external_torque(Eigen::VectorXd& tau) {
  Eigen::MatrixXd J(6, panda->getDOF());
  J.setZero();
  tau.setZero(panda->getDOF());
  for (const auto contact : panda->getContacts()) {
    if (!contact.skip() && !contact.isSelfCollision()) {
      panda->getDenseJacobian(contact.getlocalBodyIndex(),
                              contact.getPosition(), J);
      tau -= J.transpose() * contact.getImpulse()->e() / sim_.getTimeStep();
    }
  }
}

double PandaRaisimDynamics::get_object_displacement() const {
  return x_.segment<OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM)(0);
}

}  // namespace manipulation
