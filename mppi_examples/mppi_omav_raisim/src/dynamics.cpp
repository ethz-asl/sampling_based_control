/*!
 * @file     dynamics.cpp
 * @author   Matthias Studiger
 * @date     19.03.2021
 * @version  1.0
 * @brief    description
 */

#include "mppi_omav_raisim/dynamics.h"
#include <ros/package.h>
#include <string>

namespace omav_raisim {
OMAVRaisimDynamics::OMAVRaisimDynamics(const std::string &robot_description,
                                       const double dt)
    : dt_(dt), robot_description_(robot_description) {
  initialize_world(robot_description);
}

void OMAVRaisimDynamics::initialize_world(
    const std::string &robot_description) {
  // Set world and robot parameters
  sim_.setTimeStep(dt_);
  sim_.setERP(0., 0.);
  // TODO(studigem): Currently obsolete, will be replaced by articulated system,
  // since
  // this offers more functionality!
  omav = sim_.addCylinder(0.5, 0.3, 5.0);
  omav->setInertia({0.12, 0, 0, 0, 0.14, 0, 0, 0, 0.18});

  // Set dimensions
  state_dimension_ = 19;  // B_Forces(3) B_Moments(3), I_velocity(3) q(4),
                          // omega(3), I_position(3)
  input_dimension_ = 6;   // dot_Forces_des(3) dot_Moments_des(3)

  x_ = observation_t::Zero(state_dimension_);
}
mppi::DynamicsBase::observation_t OMAVRaisimDynamics::step(const input_t &u,
                                                           const double dt) {
  // Calculate the torque and thrust in body frame
  R_wb = omav->getRotationMatrix();
  b_F = {x_(0), x_(1), x_(2)};
  b_T = {x_(3), x_(4), x_(5)};
  // Manually integrate the applied force
  b_F_new = {b_F(0) + u(0) * dt_, b_F(1) + u(1) * dt_, b_F(2) + u(2) * dt_};
  b_T_new = {b_T(0) + u(3) * dt_, b_T(1) + u(4) * dt_, b_T(2) + u(5) * dt_};
  w_F_new = R_wb * b_F_new;
  w_T_new = R_wb * b_T_new;

  omav->setExternalForce(0, {0, 0, 0}, w_F_new);
  omav->setExternalTorque(0, w_T_new);

  // step
  sim_.integrate();

  omav->getPosition(omav_position);
  omav->getQuaternion(omav_quaternion);
  omav->getLinearVelocity(omav_velocity);
  omav->getAngularVelocity(omav_omega);

  x_(0) = b_F_new(0);
  x_(1) = b_F_new(1);
  x_(2) = b_F_new(2);
  x_(3) = b_T_new(0);
  x_(4) = b_T_new(1);
  x_(5) = b_T_new(2);
  x_(6) = omav_velocity(0);
  x_(7) = omav_velocity(1);
  x_(8) = omav_velocity(2);
  x_(9) = omav_quaternion(0);
  x_(10) = omav_quaternion(1);
  x_(11) = omav_quaternion(2);
  x_(12) = omav_quaternion(3);
  x_(13) = omav_omega(0);
  x_(14) = omav_omega(1);
  x_(15) = omav_omega(2);
  x_(16) = omav_position(0);
  x_(17) = omav_position(1);
  x_(18) = omav_position(2);

  return x_;
}
void OMAVRaisimDynamics::reset(const observation_t &x) {
  // internal eigen state
  x_ = x;

  raisim::Vec<3> vel = x_.segment<3>(6);
  raisim::Vec<3> omega = x_.segment<3>(13);
  omav->setVelocity(vel, omega);
  raisim::Vec<4> quaternion = x_.segment<4>(9);
  omav->setOrientation(quaternion);
  raisim::Vec<3> pos = x_.tail<3>();
  omav->setPosition(pos);
  // Rotate Thrust & Torque form body to world frame
  Eigen::Quaterniond quat = {x_(9), x_(10), x_(11), x_(12)};
  Eigen::Vector3d b_F_current = x_.head<3>();
  Eigen::Vector3d b_T_current = x_.segment<3>(3);
  Eigen::Vector3d w_F = quat * b_F_current;
  Eigen::Vector3d w_T = quat * b_T_current;
  omav->setExternalForce(0, w_F);
  omav->setExternalTorque(0, w_T);
}
mppi::DynamicsBase::input_t
OMAVRaisimDynamics::get_zero_input(const observation_t &x) {
  return DynamicsBase::input_t::Zero(get_input_dimension());
}
}  // namespace omav_raisim
