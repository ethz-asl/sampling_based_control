/*!
 * @file     dynamics.cpp
 * @author   Matthias Studiger
 * @date     19.03.2021
 * @version  1.0
 * @brief    description
 */

#include "mppi_omav_raisim/dynamics.h"
#include <ros/package.h>

namespace omav_raisim {
    OMAVRaisimDynamics::OMAVRaisimDynamics(const std::string& robot_description,
                                           const double dt)
                                           : dt_(dt){
        initialize_world(robot_description);
    }

    void OMAVRaisimDynamics::initialize_world(const std::string& robot_description) {
        // Set world and robot parameters
        sim_.setTimeStep(dt_);
        robot_description_ = robot_description;
        omav = sim_.addCylinder(0.5,0.3,4.04);
        omav->setInertia({0.078359,0,0,0,0.081797,0,0,0,0.153355});

        // Set dimensions
        state_dimension_ = 19; // B_Forces(3) B_Moments(3), I_velocity(3) q(4), omega(3), I_position(3)
        input_dimension_ = 6; // dot_Forces_des(3) dot_Moments_des(3)

        x_ = observation_t::Zero(state_dimension_);
    }
    mppi::DynamicsBase::observation_t OMAVRaisimDynamics::step(const input_t &u, const double dt) {
        //Manually integrate the applied force
        new_thrust = {x_(0) + u(0)*dt,
                      x_(1) + u(1)*dt,
                      x_(2) + u(2)*dt};
        new_torque = {x_(3) + u(3)*dt,
                       x_(4) + u(4)*dt,
                       x_(5) + u(5)*dt};
        omav->setExternalForce(0, new_thrust);
        omav->setExternalTorque(0, new_torque);

        omav->getPosition(omav_position);
        omav->getQuaternion(omav_quaternion);
        omav->getLinearVelocity(omav_velocity);
        omav->getAngularVelocity(omav_omega);
        // step
        sim_.integrate();

        x_(0) = new_thrust(0);
        x_(1) = new_thrust(1);
        x_(2) = new_thrust(2);
        x_(3) = new_torque(0);
        x_(4) = new_torque(1);
        x_(5) = new_torque(2);
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
        omav->setExternalForce(0, x_.head<3>());
        omav->setExternalTorque(0, x_.segment<3>(3));
        // TODO: Make less ugly (this is a quickfix, because otherwise it has problems due to ambiguous...)
        raisim::Vec<3> vel = x_.segment<3>(6);
        raisim::Vec<3> omega = x_.segment<3>(13);
        omav->setVelocity(vel, omega);
        raisim::Vec<4> quaternion = x_.segment<4>(9);
        omav->setOrientation(quaternion);
        raisim::Vec<3> pos = x_.tail<3>();
        omav->setPosition(pos);
    }
    mppi::DynamicsBase::input_t OMAVRaisimDynamics::get_zero_input(const observation_t &x) {
        return DynamicsBase::input_t::Zero(get_input_dimension());
    }

}

