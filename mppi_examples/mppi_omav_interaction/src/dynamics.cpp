/*!
 * @file     dynamics.cpp
 * @author   Matthias Studiger
 * @date     10.04.2021
 * @version  1.0
 * @brief    description
 */

#include "mppi_omav_interaction/dynamics.h"
#include <string>

namespace omav_interaction {
OMAVVelocityDynamics::OMAVVelocityDynamics(
    const std::string &robot_description, const std::string &object_description,
    const double dt)
    : dt_(dt), robot_description_(robot_description),
      object_description_(object_description) {
  initialize_world(robot_description, object_description);
  initialize_pd();
}

void OMAVVelocityDynamics::initialize_world(
    const std::string &robot_description,
    const std::string &object_description) {
  // Set world and robot parameters
  sim_.setTimeStep(dt_);
  sim_desired_.setTimeStep(dt_);
  sim_.setERP(0., 0.);
  sim_desired_.setERP(0.0, 0.0);
  // Initialize omav and object in odometry reset world
  omav_ = sim_.addArticulatedSystem(robot_description_, "/");
  object_ = sim_.addArticulatedSystem(object_description_, "/");
  ground = sim_.addGround(0.0, "steel");
  // Initialize omav and object in desired world
  omav_des_ = sim_desired_.addArticulatedSystem(robot_description_, "/");
  object_des_ = sim_desired_.addArticulatedSystem(object_description_, "/");

  sim_.setMaterialPairProp("rubber", "rubber", 0.001, 0.5, 0.001);
  sim_desired_.setMaterialPairProp("rubber", "rubber", 0.001, 0.5, 0.001);
  robot_dof_ = omav_->getDOF();
  // Set dimensions
  state_dimension_ = 34; // I_position(3), orientation(4), I_velocity(3),
  // B_omega(3), position_object(1), velocity_object(1), forces(3),
  // contact_state(1), I_position_desired(3), orientation_desired(4),
  // I_velocity_desired(3), B_omega_desired(3),
  // pos_desired_object(1), I_velocity_des_object(1)
  input_dimension_ =
      6; // commanded_linear_velocity_(3) commanded_angular_velocity(3)

  x_ = observation_t::Zero(state_dimension_);
}

void OMAVVelocityDynamics::initialize_pd() {
  cmd_.setZero(robot_dof_);
  cmdv_.setZero(robot_dof_);
  omav_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  omav_des_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);

  Eigen::VectorXd pGainDes(robot_dof_), dGainDes(robot_dof_);
  pGainDes << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  dGainDes << 150, 150, 150, 100, 100, 100;
  omav_des_->setPdGains(pGainDes, dGainDes);

  Eigen::VectorXd pGain(robot_dof_), dGain(robot_dof_);
  pGain << 3.0, 3.0, 3.0, 2.0, 2.0, 2.0;
  dGain << 15.0, 15.0, 15.0, 10.0, 10.0, 10.0;

  omav_->setPdGains(pGain, dGain);
}

mppi::DynamicsBase::observation_t OMAVVelocityDynamics::step(const input_t &u,
                                                             const double dt) {
  // Simulate desired system
  // Set the velocity and position target for the PD controller
  cmdv_ = u;
  cmd_ = x_.head<7>();
  // Set target of PD controller
  omav_des_->setPdTarget(cmd_, cmdv_);
  // Gravity Compensate the omav
  omav_des_->setGeneralizedForce(omav_des_->getNonlinearities());
  // step
  sim_desired_.integrate();
  // Get states after integration
  omav_des_->getState(omav_pose_des_, omav_velocity_des_);
  object_des_->getState(object_pose_des_, object_velocity_des_);
  // Simulate system that is reset with odometry
  omav_->setPdTarget(omav_pose_des_, omav_velocity_des_);
  omav_->setGeneralizedForce(omav_->getNonlinearities());
  sim_.integrate();
  omav_->getState(omav_pose_, omav_velocity_);
  object_->getState(object_pose_, object_velocity_);
  // get contact state
  double in_contact = 0;
  for (const auto &contact : omav_->getContacts()) {
    if (contact.skip())
      continue;
    in_contact = 1;
    break;
  }

  contact_force_ = get_contact_forces();

  // Assemble state
  x_.head<7>() = omav_pose_;
  x_.segment<6>(7) = omav_velocity_;
  x_.segment<1>(13) = object_pose_;
  x_.segment<1>(14) = object_velocity_;
  x_.segment<3>(15) = contact_force_.force;
  x_.segment<1>(18)(0) = in_contact;
  x_.segment<7>(19) = omav_pose_des_;
  x_.segment<6>(26) = omav_velocity_des_;
  x_.segment<1>(32) = object_pose_des_;
  x_.segment<1>(33) = object_velocity_des_;
  return x_;
}

void OMAVVelocityDynamics::reset(const observation_t &x) {
  // internal eigen state
  x_ = x;
  // reset omav and object in raisim
  omav_->setState(x_.head<7>(), x_.segment<6>(7));
  object_->setState(x_.segment<1>(13), x_.segment<1>(14));
  omav_des_->setState(x_.segment<7>(19), x_.segment<6>(26));
  object_des_->setState(x_.segment<1>(32), x_.segment<1>(33));
}

mppi::DynamicsBase::input_t
OMAVVelocityDynamics::get_zero_input(const observation_t &x) {
  return DynamicsBase::input_t::Zero(get_input_dimension());
}

force_t OMAVVelocityDynamics::get_contact_forces() {
  force_t force;
  for (const auto contact : omav_->getContacts()) {
    if (contact.skip())
      continue; /// if the contact is internal, one contact point is set to
    /// 'skip'
    if (contact.isSelfCollision())
      continue;

    force.force += contact.getContactFrame().e().transpose() *
                   contact.getImpulse()->e() / sim_.getTimeStep();
    force.position = contact.getPosition().e();
  }
  return force;
}

force_t OMAVVelocityDynamics::get_dominant_force() {
  force_t force;
  // Initialize Force struct to prevent problems when not in contact
  force.force = {0, 0, 0};
  force.position = {0, 0, 0};
  // Vector that caches the force value
  Eigen::Vector3d current_force;
  double max_force = 0.0;
  for (const auto contact : omav_->getContacts()) {
    if (contact.skip())
      continue; /// if the contact is internal, one contact point is set to
    /// 'skip'
    if (contact.isSelfCollision())
      continue;
    // Get the contact force
    current_force = contact.getContactFrame().e().transpose() *
                    contact.getImpulse()->e() / sim_.getTimeStep();
    // Function only returns the maximum force, checks if current force is
    // bigger than the on before
    if (current_force.norm() > max_force) {
      max_force = current_force.norm();
      force.force = current_force;
      force.position = contact.getPosition().e();
    }
  }
  return force;
}
} // namespace omav_interaction
