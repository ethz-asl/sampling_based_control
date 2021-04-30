/*!
 * @file     pendulumcartdynamics.h
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#pragma once

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include <mppi/core/dynamics.h>

namespace pole_cart {

enum PoleCartDim {
  STATE_DIMENSION = 4,     // x, xd, theta, theta_d
  INPUT_DIMENSION = 1,     // F
  REFERENCE_DIMENSION = 2  // x, theta
};

struct PoleCartDynamicsConfig {
  double mp = 0.5;
  double mc = 1.0;
  double l = 1.0;
  double mutheta = 0.7;
  double mux = 10.0;
  double dt_internal = 0.001;
};

class PoleCartDynamics : public mppi::Dynamics {
 public:
  PoleCartDynamics() {
    x_ = mppi::observation_t::Zero(PoleCartDim::STATE_DIMENSION);
    xd_ = mppi::observation_t::Zero(PoleCartDim::STATE_DIMENSION);
  };

  explicit PoleCartDynamics(const PoleCartDynamicsConfig& config)
      : PoleCartDynamics() {
    config_ = config;
  };

  ~PoleCartDynamics() = default;

 public:
  void set_dynamic_properties(const PoleCartDynamicsConfig& config) {
    config_ = config;
  }

  size_t get_input_dimension() override { return PoleCartDim::INPUT_DIMENSION; }
  size_t get_state_dimension() override { return PoleCartDim::STATE_DIMENSION; }

  mppi::dynamics_ptr create() override {
    return std::make_shared<PoleCartDynamics>();
  }

  mppi::dynamics_ptr clone() const override {
    return std::make_shared<PoleCartDynamics>(*this);
  }

  void reset(const mppi::observation_t& x) override;

  mppi::observation_t step(const mppi::input_t& u, const double dt) override;

  const mppi::observation_t get_state() const override;

 private:
  void compute_velocities(double F);
  void integrate_internal(double F, double dt);

 private:
  PoleCartDynamicsConfig config_;
  mppi::observation_t x_;
  mppi::observation_t xd_;
};
}  // namespace pole_cart
