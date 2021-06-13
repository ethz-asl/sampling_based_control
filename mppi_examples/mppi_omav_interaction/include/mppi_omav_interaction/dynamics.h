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

  raisim::ArticulatedSystem *get_omav() { return omav; }
  raisim::ArticulatedSystem *get_object() { return object; }

protected:
  size_t input_dimension_;
  size_t state_dimension_;
  size_t robot_dof_;

  observation_t x_;

private:
  double dt_;
  std::string robot_description_;
  std::string object_description_;

  raisim::ArticulatedSystem *omav;
  raisim::ArticulatedSystem *object;
  raisim::Ground *ground;

  raisim::World sim_;

  Eigen::VectorXd cmd, cmdv;
  Eigen::VectorXd omav_pose, omav_velocity;
  Eigen::VectorXd object_pose, object_velocity;
  force_t contact_force;
};
} // namespace omav_velocity
