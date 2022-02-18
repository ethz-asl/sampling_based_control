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
    : dt_(dt),
      robot_description_(robot_description),
      object_description_(object_description),
      input_dimension_(6),
      state_dimension_(33),
      derivative_dimension_(12),
      robot_dof_(6) {
  initialize_world(robot_description, object_description);
  initialize_pd();
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

  // Material properties: friction, bounciness (restitution), restitution threshold
  // sim_.setMaterialPairProp("rubber", "rubber", 0.001, 0.5, 0.001);
  sim_.setMaterialPairProp("steel", "steel", 0.001, 0.001, 0.5);
  sim_.setDefaultMaterial(0.001, 0.001, 0.5);
  // raisim::CollisionSet omav_c = omav_->getCollisionBodies();
  // std::cout << "Collision bodies omav: ";
  // for (auto c : omav_c) {
  //   std::cout << c.name << ", ";
  // }
  // robot_dof_ = omav_->getDOF();
  // Set dimensions
  // state_dimension_ = 32;
  // I_position(3), 0
  // orientation(4), 3
  // I_velocity(3), 7
  // B_omega(3), 10
  // position_object(1), 13
  // velocity_object(1), 14
  // forces(3), 15
  // contact_state(1), 18
  // I_position_desired(3), 19
  // orientation_desired(4), 23
  // I_velocity_desired(3), 26
  // B_omega_desired(3) 29
  // input_dimension_ = 6;  // linear_acceleration_(3) angular_acceleration(3)
  // derivative_dimension_ = 12;

  x_ = observation_t::Zero(state_dimension_);
  xd_ = observation_t::Zero(derivative_dimension_);
}

void OMAVVelocityDynamics::initialize_pd() {
  cmd_.setZero(7);
  cmdv_.setZero(6);
  omav_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);

  Eigen::VectorXd pGain(robot_dof_), dGain(robot_dof_);
  // TODO: Set gains according to real values.
  pGain << 20.0, 20.0, 20.0, 35.0, 35.0, 35.0;
  dGain << 5.0, 5.0, 5.0, 12.0, 12.0, 12.0;
  settings_.pGains = pGain;
  settings_.dGains = dGain;

  omav_->setPdGains(pGain, dGain);
}

mppi::DynamicsBase::observation_t OMAVVelocityDynamics::step(const input_t &u,
                                                             const double dt) {
  // Simulate desired system
  // Integrate the linear and angular accelerations and velocities
  integrate_internal(u, dt_);

  // Simulate system that is reset with odometry
  cmd_ = x_.segment<7>(19);
  cmdv_ = x_.segment<6>(26);
  omav_->setPdTarget(cmd_, cmdv_);
  feedforward_acceleration_ << xd_.segment<3>(6), 0.0, 0.0, 0.0;
  nonLinearities_ = omav_->getNonlinearities().e();
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
  x_.segment<6>(7) = omav_velocity_;
  x_.segment<1>(13) = object_pose_;
  x_.segment<1>(14) = object_velocity_;
  x_.segment<3>(15) = contact_force_.force;
  x_(18) = in_contact;
  x_(32) = unwanted_contact;
  return x_;
}

void OMAVVelocityDynamics::reset(const observation_t &x) {
  // internal eigen state
  x_ = x;
  // reset omav and object in raisim
  omav_->setState(x_.head<7>(), x_.segment<6>(7));
  object_->setState(x_.segment<1>(13), x_.segment<1>(14));
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
        contact.getContactFrame().e().transpose() * contact.getImpulse()->e() /
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
      // Adding the norm leads to noisy values, does not work in optimizer:
      // unwanted_contact += this_contact_force.norm();
      unwanted_contact = 1.0;
    } else {
      force.force += this_contact_force;
    }
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
      continue;  /// if the contact is internal, one contact point is set to
    /// 'skip'
    if (contact.isSelfCollision()) continue;
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

inline void OMAVVelocityDynamics::integrate_quaternion(
    const Eigen::Vector3d &omega_B, const Eigen::Quaterniond &q_WB_n,
    Eigen::Quaterniond &q_WB_n_plus_one, const double &dt) const {
  const double angVelNorm = omega_B.norm();
  if (angVelNorm == 0) {
    q_WB_n_plus_one = q_WB_n;
  } else {
    Eigen::Vector3d omega_W = q_WB_n * omega_B;
    Eigen::Quaterniond q_tilde;
    q_tilde.w() = std::cos(angVelNorm * dt / 2.0);
    q_tilde.vec() = std::sin(angVelNorm * dt / 2.0) * omega_W / angVelNorm;
    q_WB_n_plus_one = q_tilde * q_WB_n;
  }
}

void OMAVVelocityDynamics::compute_velocities(
    const mppi::DynamicsBase::input_t &u) {
  // xd_ contains the derivatives of the reference trajectory, i.e. reference
  // velocities and accelerations The input u contains the accelerations of the
  // reference trajectory
  xd_.head<6>() = x_.segment<6>(26);
  // TODO: Time derivative of the reference velocity is the input acceleration -
  // 5*reference velocity? It seems like this is done to damp the input
  // commands.
  // xd_.segment<6>(6) = u - 5 * x_.segment<6>(26);
  xd_.segment<6>(6) = u - settings_.damping * x_.segment<6>(26);
  // xd_.segment<6>(6) = u;
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

  // Integrate position and Quaternion
  x_.segment<3>(19) += xd_.head<3>() * dt;
  Eigen::Quaterniond quat_n(x_(22), x_(23), x_(24), x_(25)), quat_n_plus_one;
  integrate_quaternion(xd_.segment<3>(3), quat_n, quat_n_plus_one, dt);
  x_.segment<4>(22) << quat_n_plus_one.w(), quat_n_plus_one.vec();
  // Integrate velocities
  x_.segment<6>(26) += xd_.segment<6>(6) * dt;
}

}  // namespace omav_interaction
