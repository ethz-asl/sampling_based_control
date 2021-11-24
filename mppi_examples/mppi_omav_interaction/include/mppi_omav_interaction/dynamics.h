/*!
 * @file     dynamics.h
 * @author   Matthias Studiger
 * @date     10.04.2021
 * @version  1.0
 * @brief    description
 */

#pragma once

#include "raisim/object/ArticulatedSystem/ArticulatedSystem.hpp"
#include <raisim/World.hpp>
#include <raisim/configure.hpp>

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <memory>
#include <mppi/dynamics/dynamics_base.h>
#include <numeric>
#include <ros/package.h>
#include <stdexcept>
#include <string>

namespace omav_interaction {

struct force_t {
  Eigen::Vector3d force;
  Eigen::Vector3d position;
};

class OMAVVelocityDynamics : public mppi::DynamicsBase {
public:
  OMAVVelocityDynamics(const std::string &robot_description,
                       const std::string &object_description, const double dt);

  ~OMAVVelocityDynamics() = default;

private:
  void initialize_world(const std::string &robot_description,
                        const std::string &object_description);

  void initialize_pd();

  void integrate_quaternion(const Eigen::Vector3d omega,
                            const Eigen::Quaterniond q_n,
                            Eigen::Quaterniond &q_n_plus_one);
  void compute_velocities(const input_t &u);
  void integrate_internal(const input_t &u, double dt);

public:
  double get_dt() { return dt_; }

  size_t get_input_dimension() override { return input_dimension_; }

  size_t get_state_dimension() override { return state_dimension_; }

  dynamics_ptr create() override {
    return std::make_shared<OMAVVelocityDynamics>(robot_description_,
                                                  object_description_, dt_);
  }

  dynamics_ptr clone() const override {
    std::cout << "cannot clone, raisim world copy constructor is deleted. "
                 "Returning empty pointer"
              << std::endl;
    return dynamics_ptr();
  }

  void reset(const observation_t &x) override;

  observation_t step(const input_t &u, const double dt) override;

  input_t get_zero_input(const observation_t &x) override;

  const observation_t get_state() const override { return x_; }

  force_t get_contact_forces();
  force_t get_dominant_force();

  raisim::World *get_world() { return &sim_; }

  raisim::ArticulatedSystem *get_omav() { return omav_; }
  raisim::ArticulatedSystem *get_object() { return object_; }

protected:
  size_t input_dimension_;
  size_t state_dimension_;
  size_t derrivative_dimension_;
  size_t robot_dof_;

  observation_t x_;
  observation_t xd_;

private:
  double dt_;
  std::string robot_description_;
  std::string object_description_;

  // Objects in odometry simulation
  raisim::ArticulatedSystem *omav_;
  raisim::ArticulatedSystem *object_;
  raisim::Ground *ground;

  raisim::World sim_;

  Eigen::VectorXd cmd_, cmdv_;
  Eigen::Matrix<double, 6, 1> feedforward_force_;
  Eigen::Matrix<double, 6, 1> feedforward_acceleration_;
  Eigen::Matrix<double, 6, 1> feedforward_gravity_;
  Eigen::Matrix<double, 6, 1> nonLinearities_;
  Eigen::VectorXd omav_pose_, omav_velocity_;
  Eigen::VectorXd object_pose_, object_velocity_;
  force_t contact_force_;
};
} // namespace omav_interaction
