/*!
 * @file     dynamics.cpp
 * @author   Matthias Studiger
 * @date     10.04.2021
 * @version  1.0
 * @brief    description
 */

#include "mppi_omav_interaction/dynamics.h"

namespace omav_interaction {
OMAVVelocityDynamics::OMAVVelocityDynamics(
    const std::string &robot_description, const std::string &object_description,
    const double dt)
    : dt_(dt),
      robot_description_(robot_description),
      object_description_(object_description),
      input_dimension_(control_input_description::SIZE_CONTROL_INPUT),
      state_dimension_(
          omav_state_description_simulation::SIZE_OMAV_STATE_SIMULATION),
      derivative_dimension_(12),
      robot_dof_(6) {
  initialize_world(robot_description, object_description);
  initialize_pd();

#ifdef RAISIM_VISUALIZATION_SERVER
  server.launchServer();
#endif
}

void OMAVVelocityDynamics::initialize_world(
    const std::string &robot_description,
    const std::string &object_description) {
  // Set world and robot parameters
  sim_.setTimeStep(dt_);
  sim_.setERP(0., 0.);
  // Initialize omav and object in odometry reset world
  omav_ = sim_.addArticulatedSystem(robot_description_, "/");
  object_ = sim_.addArticulatedSystem(object_description_, "/");
  ground = sim_.addGround(0.0, "steel");

  // Material properties: friction, bounciness, restitution threshold
  sim_.setMaterialPairProp("steel", "steel", 0.001, 0.5, 0.001);
  sim_.setDefaultMaterial(0.001, 0.001, 0.001);

  x_ = observation_t::Zero(state_dimension_);
  xd_ = observation_t::Zero(derivative_dimension_);
}

void OMAVVelocityDynamics::initialize_pd() {
  cmdv_.setZero(6);
  omav_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);

  settings_.pGains.setZero();
  settings_.dGains << 5.0, 5.0, 5.0, 12.0, 12.0, 12.0;

  omav_->setPdGains(settings_.pGains, settings_.dGains);
}

mppi::DynamicsBase::observation_t OMAVVelocityDynamics::step(const input_t &u,
                                                             const double dt) {
  // Simulate desired system
  // Integrate the linear and angular accelerations and velocities
  integrate_internal(u, dt_);

  cmdv_ = x_.segment<6>(
      omav_state_description_simulation::MAV_LINEAR_VELOCITY_X_DESIRED_WORLD);
  omav_->setPdTarget(Eigen::VectorXd::Zero(7), cmdv_);
  feedforward_acceleration_ << xd_.segment<3>(6), 0.0, 0.0, 0.0;
  nonLinearities_ = omav_->getNonlinearities(sim_.getGravity()).e();
  feedforward_force_ =
      feedforward_acceleration_ * settings_.mass + nonLinearities_;
  // feedforward_force_ = nonLinearities_;
  omav_->setGeneralizedForce(feedforward_force_);
  sim_.integrate();
  omav_->getState(omav_pose_, omav_velocity_);
  object_->getState(object_pose_, object_velocity_);
  // get contact state
  double in_contact = 0.0;
  for (const auto &contact : omav_->getContacts()) {
    if (contact.skip()) continue;
    in_contact = 1;
    break;
  }
  double unwanted_contact = 0.0;
  contact_force_ = get_contact_forces(unwanted_contact);

  // Assemble state
  x_.head<7>() = omav_pose_;
  x_.segment<6>(
      omav_state_description_simulation::MAV_LINEAR_VELOCITY_X_WORLD) =
      omav_velocity_;
  x_.segment<1>(omav_state_description_simulation::OBJECT_HINGE_ORIENTATION) =
      object_pose_;
  x_.segment<1>(omav_state_description_simulation::OBJECT_HINGE_VELOCITY) =
      object_velocity_;
  x_.segment<3>(omav_state_description_simulation::INTERACTION_FORCE_X) =
      contact_force_.force;
  x_(omav_state_description_simulation::CONTACT_STATE) = in_contact;
  x_(omav_state_description_simulation::UNWANTED_CONTACT) = unwanted_contact;
  return x_;
}

void OMAVVelocityDynamics::reset(const observation_t &x) {
  x_ = x;

  // reset omav and object in raisim
  omav_->setState(
      x_.head<7>(),
      x_.segment<6>(
          omav_state_description_simulation::MAV_LINEAR_VELOCITY_X_WORLD));

  Eigen::Vector3d base_position = x_.segment<3>(
      omav_state_description_simulation::OBJECT_BASE_POSITION_X_WORLD);
  raisim::Vec<4> base_orientation = {
      x(omav_state_description_simulation::OBJECT_BASE_ORIENTATION_W_WORLD),
      x(omav_state_description_simulation::OBJECT_BASE_ORIENTATION_X_WORLD),
      x(omav_state_description_simulation::OBJECT_BASE_ORIENTATION_Y_WORLD),
      x(omav_state_description_simulation::OBJECT_BASE_ORIENTATION_Z_WORLD)};

  object_->setBasePos_e(base_position);
  object_->setBaseOrientation(base_orientation);

  object_->setState(
      x_.segment<1>(
          omav_state_description_simulation::OBJECT_HINGE_ORIENTATION),
      x_.segment<1>(omav_state_description_simulation::OBJECT_HINGE_VELOCITY));
}

OMAVVelocityDynamics::observation_t
OMAVVelocityDynamics::get_extended_state_from_observation(
    const observation_t &x) const {
  observation_t extended_state(static_cast<int>(
      omav_state_description_simulation::SIZE_OMAV_STATE_SIMULATION));
  extended_state.setZero();
  extended_state.head<omav_state_description::SIZE_OMAV_STATE>() = x;

  // possible to reset raisim object here to get correct transforms
  Eigen::Vector3d base_position;
  base_position << -0.242, -0.595, 0.834;
  Eigen::Vector4d base_orientation;
  base_orientation << 0.680391212677159, 0, 0.0, 0.7328490961389699;

  extended_state.segment<3>(
      omav_state_description_simulation::OBJECT_BASE_POSITION_X_WORLD) =
      base_position;
  extended_state.segment<4>(
      omav_state_description_simulation::OBJECT_BASE_ORIENTATION_W_WORLD) =
      base_orientation;

  return extended_state;
}

mppi::DynamicsBase::input_t OMAVVelocityDynamics::get_zero_input(
    const observation_t &x) {
  return DynamicsBase::input_t::Zero(get_input_dimension());
}

force_t OMAVVelocityDynamics::get_contact_forces(double &unwanted_contact) {
  force_t force;
  force.force = {0, 0, 0};
  for (const auto contact : omav_->getContacts()) {
    if (contact.skip())
      continue;  // if the contact is internal, one contact point is set to
                 // 'skip'
    if (contact.isSelfCollision()) continue;
    Eigen::Vector3d this_contact_force =
        contact.getContactFrame().e().transpose() * contact.getImpulse().e() /
        sim_.getTimeStep();
    // force.position = contact.getPosition().e();
    // Compute contact point in body frame:
    // omav_pose: q = [w, x, y, z]
    // Eigen::Quaternion expects [w, x, y, z]
    // Eigen::Quaterniond(omav_pose_.segment<4>(3)) will result in wrong
    // quaternion as it directly writes internally into [x y z w]!
    Eigen::Quaterniond q_IB = Eigen::Quaterniond(omav_pose_(3), omav_pose_(4),
                                                 omav_pose_(5), omav_pose_(6));
    Eigen::Vector3d contact_point_B =
        q_IB.inverse() * (contact.getPosition().e() - omav_pose_.head(3));
    // Check position of contact: If not at end effector, set to true.
    if (contact_point_B(0) < 0.55) {
      //   // Adding the norm leads to noisy values, does not work in optimizer:
      //   // unwanted_contact += this_contact_force.norm();
      unwanted_contact = 1.0;
    }  // else {
    force.force += this_contact_force;
    // }
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
      continue;  // if the contact is internal, one contact point is set to
                 // 'skip'
    if (contact.isSelfCollision()) continue;
    // Get the contact force
    current_force = contact.getContactFrame().e().transpose() *
                    contact.getImpulse().e() / sim_.getTimeStep();
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

void OMAVVelocityDynamics::compute_velocities(
    const mppi::DynamicsBase::input_t &u) {
  // xd_ contains the derivatives of the reference trajectory, i.e. reference
  // velocities and accelerations The input u contains the accelerations of the
  // reference trajectory
  xd_.head<6>() = x_.segment<6>(
      omav_state_description_simulation::MAV_LINEAR_VELOCITY_X_DESIRED_WORLD);

  xd_.segment<6>(6) =
      u - settings_.damping *
              x_.segment<6>(omav_state_description_simulation::
                                MAV_LINEAR_VELOCITY_X_DESIRED_WORLD);
}

void OMAVVelocityDynamics::integrate_internal(
    const mppi::DynamicsBase::input_t &u, double dt) {
  if (dt > dt_) {
    std::stringstream ss;
    ss << "Integrate internal called with dt larger that internal dt: " << dt
       << "> " << dt_;
    throw std::runtime_error(ss.str());
  }

  compute_velocities(u);

  // Integrate velocities
  x_.segment<6>(
      omav_state_description_simulation::MAV_LINEAR_VELOCITY_X_DESIRED_WORLD) +=
      xd_.segment<6>(6) * dt;
}

}  // namespace omav_interaction
