/*!
 * @file     dynamics.cpp
 * @author   Matthias Studiger
 * @date     19.03.2021
 * @version  1.0
 * @brief    description
 */

#include "mppi_omav_raisim/dynamics.h"
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
  // Initialize omav
  robot_description_ = robot_description;
  omav = sim_.addArticulatedSystem(robot_description_, "/");

  // Set dimensions
  state_dimension_ = 19;  // B_Forces(3) B_Moments(3), I_velocity(3) q(4),
                          // omega(3), I_position(3)
  input_dimension_ = 6;   // dot_Forces_des(3) dot_Moments_des(3)

  x_ = observation_t::Zero(state_dimension_);
}
mppi::DynamicsBase::observation_t OMAVRaisimDynamics::step(const input_t &u,
                                                           const double dt) {
  // Calculate the torque and thrust in body frame, input rates
  w_F_new = {x_(0) + u(0), x_(1) + u(1), x_(2) + u(2)};
  //w_T_new = {x_(3) + u(3), x_(4) + u(4), x_(5) + u(5)};
  // New Forces and Torques as input
  //w_F_new = {u(0), u(1), u(2)};
  w_T_new = {u(3), u(4), u(5)};
  omav->setGeneralizedForce(omav->getNonlinearities());
  omav->setExternalForce(0, w_F_new);
  omav->setExternalTorque(0, w_T_new);

  // step
  sim_.integrate();

  omav->getState(omav_pose, omav_velocities);

  x_.head<3>() = w_F_new;
  x_.segment<3>(3) = w_T_new;
  x_.segment<3>(6) = omav_velocities.head<3>();
  x_.segment<4>(9) = omav_pose.tail<4>();
  x_.segment<3>(13) = omav_velocities.tail<3>();
  x_.segment<3>(16) = omav_pose.head<3>();

  return x_;
}
void OMAVRaisimDynamics::reset(const observation_t &x) {
  // internal eigen state
  x_ = x;
  // reset omav in raisim
  Eigen::VectorXd omav_state_pose(7);
  omav_state_pose << x_.tail<3>(), x_.segment<4>(9);
  Eigen::VectorXd omav_state_velocity(6);
  omav_state_velocity << x_.segment<3>(6), x_.segment<3>(13);
  omav->setState(omav_state_pose, omav_state_velocity);
}
mppi::DynamicsBase::input_t
OMAVRaisimDynamics::get_zero_input(const observation_t &x) {
  return DynamicsBase::input_t::Zero(get_input_dimension());
}
}  // namespace omav_raisim
