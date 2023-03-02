/*!
 * @file     dynamics.h
 * @author   Matthias Studiger
 * @date     10.04.2021
 * @version  1.0
 * @brief    description
 */

#pragma once

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>

#include <raisim/World.hpp>
#include <raisim/configure.hpp>
#include <raisim/object/ArticulatedSystem/ArticulatedSystem.hpp>

#ifndef NDEBUG
#include <raisim/RaisimServer.hpp>
#endif

#include <mppi/dynamics/dynamics_base.h>
#include <mppi_omav_interaction/omav_interaction_common.h>

namespace omav_interaction {

struct force_t {
  Eigen::Vector3d force;
  Eigen::Vector3d position;
};

struct OmavDynamicsSettings {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double mass = 4.337;
  double damping = 5.0;
  Eigen::Matrix<double, 6, 1> pGains;
  Eigen::Matrix<double, 6, 1> dGains;
};

class OMAVVelocityDynamics : public mppi::DynamicsBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using omav_dynamics_ptr = std::shared_ptr<OMAVVelocityDynamics>;
  OMAVVelocityDynamics(const std::string &robot_description,
                       const std::string &object_description, const double dt);

  ~OMAVVelocityDynamics() = default;

 private:
  void initialize_world(const std::string &robot_description,
                        const std::string &object_description);

  void initialize_pd();

  void compute_velocities(const input_t &u);
  void integrate_internal(const input_t &u, double dt);

 public:
  double get_dt() { return dt_; }

  size_t get_input_dimension() override { return input_dimension_; }

  size_t get_state_dimension() override { return state_dimension_; }

  void setDGains(const Eigen::Matrix<double, 6, 1> &d) {
    settings_.pGains.setZero();
    settings_.dGains = d;

    omav_->setPdGains(settings_.pGains, settings_.dGains);
  }

  void setDampingFactor(const double &k) { settings_.damping = k; }

  void setMass(const double &m) { settings_.mass = m; }

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
  observation_t get_extended_state_from_observation(
      const observation_t &x) const override;

  force_t get_contact_forces(double &unwanted_contact);
  force_t get_contact_forces() {
    double a;
    return get_contact_forces(a);
  };
  force_t get_dominant_force();

  raisim::World *get_world() { return &sim_; }

  raisim::ArticulatedSystem *get_omav() { return omav_; }
  raisim::ArticulatedSystem *get_object() { return object_; }

 protected:
  const size_t input_dimension_;
  const size_t state_dimension_;
  const size_t derivative_dimension_;
  const size_t robot_dof_;

  observation_t x_;
  observation_t xd_;

 private:
  const double dt_;
  std::string robot_description_;
  std::string object_description_;

  // Objects in raisim simulation
  raisim::ArticulatedSystem *omav_;
  raisim::ArticulatedSystem *object_;
  raisim::Ground *ground;

  raisim::World sim_;

#ifndef NDEBUG
  raisim::RaisimServer server{&sim_};
#endif

  Eigen::VectorXd cmdv_;
  Eigen::Matrix<double, 6, 1> feedforward_force_;
  Eigen::Matrix<double, 6, 1> feedforward_acceleration_;
  Eigen::Matrix<double, 6, 1> feedforward_gravity_;
  Eigen::Matrix<double, 6, 1> nonLinearities_;
  Eigen::VectorXd omav_pose_, omav_velocity_;
  Eigen::VectorXd object_pose_, object_velocity_;
  force_t contact_force_;

  OmavDynamicsSettings settings_;
};
}  // namespace omav_interaction
