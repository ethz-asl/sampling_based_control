/*!
 * @file     dynamics.cpp
 * @author   Matthias Studiger
 * @date     10.04.2021
 * @version  1.0
 * @brief    description
 */

#include "mppi_omav_interaction/dynamics.h"
#include "raisim/object/ArticulatedSystem/ArticulatedSystem.hpp"
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
  sim_.setERP(0., 0.);
  // Initialize omav
  robot_description_ = robot_description;
  omav = sim_.addArticulatedSystem(robot_description_, "/");
  object = sim_.addArticulatedSystem(object_description_, "/");
  ground = sim_.addGround(0.0, "steel");
  sim_.setMaterialPairProp("rubber", "rubber", 0.001, 0.05, 0.001);
  robot_dof_ = omav->getDOF();
  // Set dimensions
  state_dimension_ = 19; // I_position(3), orientation(4), I_velocity(3),
  // I_omega(3), I_position_object(1), I_velocity_object(1), forces(3),
  // contact_state
  input_dimension_ =
      6; // commanded_linear_velocity_(3) commanded_angular_velocity(3)

  x_ = observation_t::Zero(state_dimension_);
}

void OMAVVelocityDynamics::initialize_pd() {
  cmd.setZero(robot_dof_);
  cmdv.setZero(robot_dof_);
  omav->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  Eigen::VectorXd p_gain(6), d_gain(6);
  p_gain << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  d_gain << 10, 10, 10, 5, 5, 5;
  omav->setPdGains(p_gain, d_gain);
}

mppi::DynamicsBase::observation_t OMAVVelocityDynamics::step(const input_t &u,
                                                             const double dt) {
  // Set the velocity and position target for the PD controller
  cmdv = u;
  cmd = x_.head<7>();
  // Set target of PD controller
  omav->setPdTarget(cmd, cmdv);
  // Gravity Compensate the omav
  omav->setGeneralizedForce(omav->getNonlinearities());
  // step
  sim_.integrate();
  // Get states after integration
  omav->getState(omav_pose, omav_velocity);
  object->getState(object_pose, object_velocity);

  // get contact state
  double in_contact = 0;
  for (const auto &contact : omav->getContacts()) {
    if (contact.skip())
      continue;
    in_contact = 1;
    break;
  }

  contact_force = get_dominant_force();

  // Assemble state
  x_.head<7>() = omav_pose;
  x_.segment<6>(7) = omav_velocity;
  x_.segment<1>(13) = object_pose;
  x_.segment<1>(14) = object_velocity;
  x_.segment<3>(15) = contact_force.force;
  x_.segment<1>(18)(0) = in_contact;
  return x_;
}

void OMAVVelocityDynamics::reset(const observation_t &x) {
  // internal eigen state
  x_ = x;
  // reset omav and object in raisim
  omav->setState(x_.head<7>(), x_.segment<6>(7));
  object->setState(x_.segment<1>(13), x_.segment<1>(14));
}

mppi::DynamicsBase::input_t
OMAVVelocityDynamics::get_zero_input(const observation_t &x) {
  return DynamicsBase::input_t::Zero(get_input_dimension());
}

std::vector<force_t> OMAVVelocityDynamics::get_contact_forces() {
  std::vector<force_t> forces;
  for (const auto contact : omav->getContacts()) {
    if (contact.skip())
      continue; /// if the contact is internal, one contact point is set to
    /// 'skip'
    if (contact.isSelfCollision())
      continue;
    force_t force;
    force.force = contact.getContactFrame().e().transpose() *
                  contact.getImpulse()->e() / sim_.getTimeStep();
    force.position = contact.getPosition().e();
    forces.push_back(force);
  }
  return forces;
}

force_t OMAVVelocityDynamics::get_dominant_force() {
  force_t force;
  // Initialize Force struct to prevent problems when not in contact
  force.force = {0, 0, 0};
  force.position = {0, 0, 0};
  // Vector that caches the force value
  Eigen::Vector3d current_force;
  double max_force = 0.0;
  for (const auto contact : omav->getContacts()) {
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
