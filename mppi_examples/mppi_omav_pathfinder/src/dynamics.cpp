/*!
 * @file     omav_dynamics.cpp
 * @author   Matthias Studiger
 * @date     15.03.2021
 * @version  1.0
 * @brief    description
 */

#include "mppi_omav_pathfinder/dynamics.h"
#include <Eigen/Geometry>

using namespace mppi;
namespace omav_pathfinder {

void OMAV_PathfinderDynamics::compute_velocities(double B_f_dot_x, double B_f_dot_y, double B_f_dot_z,
                                                 double B_tau_dot_x, double B_tau_dot_y, double B_tau_dot_z) {
    // Set the inertia and inverse inertial Matrix
    //TODO: Put this in the config file
    Eigen::Matrix3d I_b_inv;
    I_b_inv << 1/config_.Ix, 0, 0,
    0, 1/config_.Iy, 0,
    0, 0, 1/config_.Iz;
    Eigen::Matrix3d I_b;
    I_b_inv << config_.Ix, 0, 0,
            0, config_.Iy, 0,
            0, 0, config_.Iz;
    // Set the force vector
    Eigen::Vector3d B_all_F(x_(0), x_(1), x_(2));
    // Set the torque vector
    Eigen::Vector3d B_all_tau(x_(3), x_(4), x_(5));
    // Calculation of the rotation matrix from the quaternion
    Eigen::Quaternion<double> q(x_(9), x_(10), x_(11), x_(12));
    q.normalize();
    Eigen::Matrix3d R_IB = q.toRotationMatrix();
    // Calculate the derivatives form the quaternions
    Eigen::Matrix4d M_l_quat;
    M_l_quat << x_(9), -x_(10), -x_(11), -x_(12),
    x_(10), x_(9), -x_(12), x_(11),
    x_(11), x_(12), x_(9), -x_(10),
    x_(12), -x_(11), x_(10), x_(9);
    Eigen::Vector4d omega_extended(0, x_(13), x_(14), x_(15));
    Eigen::Vector3d I_all_F = R_IB * B_all_F;
    Eigen::Vector4d quaternions_derivatives;
    quaternions_derivatives = 0.5*M_l_quat*omega_extended;
    // Calculate the omega derivatives
    Eigen::Vector3d omega(x_(13), x_(14), x_(15));
    Eigen::Vector3d omega_derivatives;
    omega_derivatives = I_b_inv*(B_all_tau - omega.cross(I_b*omega));

    xd_(0) = B_f_dot_x; // B_f_x
    xd_(1) = B_f_dot_y; // B_f_y
    xd_(2) = B_f_dot_z; // B_f_z
    xd_(3) = B_tau_dot_x; // B_tau_x
    xd_(4) = B_tau_dot_y; // B_tau_y
    xd_(5) = B_tau_dot_z; // B_tau_z
    xd_(6) = 1/config_.mass*I_all_F(0); // I_v_x
    xd_(7) = 1/config_.mass*I_all_F(1); // I_v_y
    xd_(8) = 1/config_.mass*I_all_F(2) - config_.gravity; // I_v_z
    xd_(9) = quaternions_derivatives(0); // q_1
    xd_(10) = quaternions_derivatives(1); // q_2
    xd_(11) = quaternions_derivatives(2); // q_3
    xd_(12) = quaternions_derivatives(3); // q_4
    xd_(13) = omega_derivatives(0); // omega_x
    xd_(14) = omega_derivatives(1); // omega_y
    xd_(15) = omega_derivatives(2); // omega_z
    xd_(16) = x_(6); // I_x
    xd_(17) = x_(7); // I_y
    xd_(18) = x_(8); // I_z
}

void OMAV_PathfinderDynamics::integrate_internal(double B_f_dot_x, double B_f_dot_y, double B_f_dot_z,
                                                 double B_tau_dot_x, double B_tau_dot_y, double B_tau_dot_z,
                                                 double dt){
  if (dt > config_.dt_internal) {
    std::stringstream ss;
    ss << "Integrate internal called with dt larger that internal dt: " << dt
       << "> " << config_.dt_internal;
    throw std::runtime_error(ss.str());
  }

  compute_velocities(B_f_dot_x, B_f_dot_y, B_f_dot_z, B_tau_dot_x, B_tau_dot_y, B_tau_dot_z);
  x_ += xd_ * dt;
}

DynamicsBase::observation_t OMAV_PathfinderDynamics::step(
    const DynamicsBase::input_t &u, const double dt) {
  size_t steps = std::floor(dt / config_.dt_internal);
  if (steps > 0) {
    for (size_t i = 0; i < steps; i++)
      integrate_internal(u(0), u(1), u(2), u(3), u(4), u(5), config_.dt_internal);
  }
  double dt_last = dt - steps * config_.dt_internal;
  integrate_internal(u(0), u(1), u(2), u(3), u(4), u(5), dt_last);
  return x_;
}

const DynamicsBase::observation_t OMAV_PathfinderDynamics::get_state() const {
  return x_;
}

void OMAV_PathfinderDynamics::reset(const DynamicsBase::observation_t &x) { x_ = x; }
}  // namespace omav_pathfinder
