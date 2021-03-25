/*!
 * @file     dynamics.h
 * @author   Matthias Studiger
 * @date     19.03.2021
 * @version  1.0
 * @brief    description
 */

#pragma once

#include <raisim/World.hpp>
#include <raisim/configure.hpp>

#include <mppi/dynamics/dynamics_base.h>
#include <ros/package.h>
#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <numeric>
#include <stdexcept>

namespace omav_raisim {
    struct force_t {
        Eigen::Vector3d force;
        Eigen::Vector3d position;
    };

    class OMAVRaisimDynamics : public mppi::DynamicsBase {
    public:
        OMAVRaisimDynamics(const std::string& robot_description, const double dt);
        ~OMAVRaisimDynamics() = default;

    private:
        void initialize_world();

    public:
        double get_dt() { return dt_; }
        size_t get_input_dimension() override {return input_dimension_; }
        size_t get_state_dimension() override {return state_dimension_; }
        dynamics_ptr create() override {
            return std::make_shared<OMAVRaisimDynamics>(
                    robot_description_, dt_
                    );
        }

        dynamics_ptr clone() const override {
            std::cout << "cannot clone, raisim world copy constructor is deleted. "
                         "Returning empty pointer"
                         << std::endl;
            return dynamics_ptr();
        }

        void reset(const observation_t& x) override;

        observation_t step(const input_t& u, const double dt) override;
        input_t get_zero_input(const observation_t& x) override;
        const observation_t get_state() const override { return x_; }

        raisim::World* get_world() { return &sim_; }
        raisim::Cylinder* get_omav() { return omav; }

    protected:
        size_t input_dimension_;
        size_t state_dimension_;

        observation_t x_;
    private:
        double dt_;
        std::string robot_description_;

        raisim::Cylinder* omav;

        raisim::World sim_;

        Eigen::Vector3d new_thrust_body_frame;
        Eigen::Vector3d new_torque_body_frame;

        Eigen::Matrix3d current_rotation_matrix;
        Eigen::Vector3d new_thrust_world_frame;
        Eigen::Vector3d thrust_body_frame;
        Eigen::Vector3d new_torque_world_frame;
        Eigen::Vector3d torque_body_frame;

        raisim::Vec<3> omav_velocity;
        raisim::Vec<4> omav_quaternion;
        raisim::Vec<3> omav_omega;
        raisim::Vec<3> omav_position;
    };
}
