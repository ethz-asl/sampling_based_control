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

PandaRaisimDynamics::PandaRaisimDynamics(const std::string& robot_description,
                                         const std::string& object_description,
                                         const double dt, const bool fixed_base,
                                         const PandaRaisimGains& gains)
    : fixed_base_(fixed_base), dt_(dt), gains_(gains) {
  initialize_world(robot_description, object_description);
  initialize_pd();
  set_collision();
};

void PandaRaisimDynamics::initialize_world(
    const std::string& robot_description,
    const std::string& object_description) {
  sim_.setTimeStep(dt_);
  sim_.setERP(0., 0.);
  sim_.setMaterialPairProp("steel", "steel", 0.01, 0.0, 0.0);
  robot_description_ = robot_description;
  panda = sim_.addArticulatedSystem(robot_description_, "/");
  panda->setGeneralizedForce(Eigen::VectorXd::Zero(panda->getDOF()));

  /// create raisim objects
  object_description_ = object_description;
  object = sim_.addArticulatedSystem(object_description_, "/");
  object->setGeneralizedForce(Eigen::VectorXd::Zero(object->getDOF()));
  object->setGeneralizedCoordinate(Eigen::VectorXd::Zero(1));

  // robot dof and base check
  if (fixed_base_) {
    robot_dof_ = ARM_GRIPPER_DIM;
    state_dimension_ = 2 * (ARM_GRIPPER_DIM + OBJECT_DIMENSION) + CONTACT_STATE;
    input_dimension_ = ARM_GRIPPER_DIM - 1;  // mimic joint for gripper
  } else {
    robot_dof_ = BASE_ARM_GRIPPER_DIM;
    state_dimension_ =
        2 * (BASE_ARM_GRIPPER_DIM + OBJECT_DIMENSION) + CONTACT_STATE;
    input_dimension_ = BASE_ARM_GRIPPER_DIM - 1;  // mimic joint for gripper
  }
  x_ = observation_t::Zero(state_dimension_);
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

  if (!fixed_base_) {
    joint_p_gain.head(BASE_DIMENSION) =
        gains_.base_gains.Kp;  //.setConstant(0);
    joint_d_gain.head(BASE_DIMENSION) =
        gains_.base_gains.Kd;  //.setConstant(1000.0);
    joint_p_gain.segment(BASE_DIMENSION, ARM_DIMENSION) =
        gains_.arm_gains.Kp;  //.setConstant(0.0);
    joint_d_gain.segment(BASE_DIMENSION, ARM_DIMENSION) =
        gains_.arm_gains.Kd;  //.setConstant(10.0);
  } else {
    joint_p_gain.head(ARM_DIMENSION) = gains_.arm_gains.Kp;  //.setConstant(0);
    joint_d_gain.head(ARM_DIMENSION) =
        gains_.arm_gains.Kd;  //.setConstant(10.0);
  }

  joint_p_gain.tail(GRIPPER_DIMENSION) =
      gains_.gripper_gains.Kp;  //.setConstant(100);
  joint_d_gain.tail(GRIPPER_DIMENSION) =
      gains_.gripper_gains.Kd;  //.setConstant(50.0);

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

mppi::DynamicsBase::observation_t PandaRaisimDynamics::step(
    const DynamicsBase::input_t& u, const double dt) {
  // keep the gripper in the current position
  if (fixed_base_) {
    cmd.tail<PandaDim::GRIPPER_DIMENSION>()
        << x_.head<ARM_GRIPPER_DIM>().tail<GRIPPER_DIMENSION>();
  } else {
    cmd.tail<PandaDim::GRIPPER_DIMENSION>()
        << x_.head<BASE_ARM_GRIPPER_DIM>().tail<GRIPPER_DIMENSION>();
  }

  if (fixed_base_) {
    cmdv.head<ARM_DIMENSION>() = u.head<ARM_DIMENSION>();
  } else {
    cmdv(0) = u(0) * std::cos(x_(2)) - u(1) * std::sin(x_(2));
    cmdv(1) = u(0) * std::sin(x_(2)) + u(1) * std::cos(x_(2));
    cmdv(2) = u(2);
    cmdv.segment<ARM_DIMENSION>(BASE_DIMENSION) =
        u.segment<ARM_DIMENSION>(BASE_DIMENSION);
  }

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

  // step simulation
  sim_.integrate();

  if (fixed_base_) {
    x_.head<ARM_GRIPPER_DIM>() = joint_p;
    x_.segment<ARM_GRIPPER_DIM>(ARM_GRIPPER_DIM) = joint_v;
    x_.segment<2 * OBJECT_DIMENSION>(2 * ARM_GRIPPER_DIM)(0) = object_p(0);
    x_.segment<2 * OBJECT_DIMENSION>(2 * ARM_GRIPPER_DIM)(1) = object_v(0);
    x_.tail<1>()(0) = in_contact;
  } else {
    x_.head<BASE_ARM_GRIPPER_DIM>() = joint_p;
    x_.segment<BASE_ARM_GRIPPER_DIM>(BASE_ARM_GRIPPER_DIM) = joint_v;
    x_.segment<2 * OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM)(0) = object_p(0);
    x_.segment<2 * OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM)(1) = object_v(0);
    x_.tail<1>()(0) = in_contact;
  }
  return x_;
}

void PandaRaisimDynamics::reset(const DynamicsBase::observation_t& x) {
  // internal eigen state
  x_ = x;

  // reset arm
  if (fixed_base_) {
    panda->setState(x_.head<ARM_GRIPPER_DIM>(),
                    x_.segment<ARM_GRIPPER_DIM>(ARM_GRIPPER_DIM));
    object->setState(x_.segment<OBJECT_DIMENSION>(2 * ARM_GRIPPER_DIM),
                     x_.segment<OBJECT_DIMENSION>(2 * ARM_GRIPPER_DIM + 1));
  } else {
    panda->setState(x_.head<BASE_ARM_GRIPPER_DIM>(),
                    x_.segment<BASE_ARM_GRIPPER_DIM>(BASE_ARM_GRIPPER_DIM));
    object->setState(
        x_.segment<OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM),
        x_.segment<OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM + 1));
  }
}

mppi::DynamicsBase::input_t PandaRaisimDynamics::get_zero_input(
    const observation_t& x) {
  return DynamicsBase::input_t::Zero(get_input_dimension());
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
  for (const auto contact : object->getContacts()) {
    if (contact.skip())
      continue;  /// if the contact is internal, one contact point is set to
                 /// 'skip'
    if (contact.isSelfCollision()) continue;
    force_t force;
    force.force = contact.getContactFrame().e() * contact.getImpulse()->e() /
                  sim_.getTimeStep();
    force.position = contact.getPosition().e();
    forces.push_back(force);
  }
  return forces;
}

double PandaRaisimDynamics::get_object_displacement() const {
  if (fixed_base_) return x_.segment<OBJECT_DIMENSION>(2 * ARM_GRIPPER_DIM)(0);
  return x_.segment<OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM)(0);
}

}  // namespace manipulation
