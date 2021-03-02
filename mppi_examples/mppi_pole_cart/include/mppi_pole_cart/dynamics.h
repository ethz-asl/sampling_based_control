/*!
 * @file     pendulumcartdynamics.h
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#pragma once

#include <math.h>
#include <mppi/dynamics/dynamics_base.h>
#include <Eigen/Core>
#include <iostream>
#include <stdexcept>

namespace pole_cart {

enum PoleCartDim {
  STATE_DIMENSION = 4,     // x, xd, theta, thetad
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

class PoleCartDynamics : public mppi::DynamicsBase {
 public:
  PoleCartDynamics() {
    x_ = observation_t::Zero(PoleCartDim::STATE_DIMENSION);
    xd_ = observation_t::Zero(PoleCartDim::STATE_DIMENSION);
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

  dynamics_ptr create() override {
    return std::make_shared<PoleCartDynamics>();
  }

  dynamics_ptr clone() const override {
    return std::make_shared<PoleCartDynamics>(*this);
  }

  void reset(const observation_t& x) override;

  observation_t step(const input_t& u, const double dt) override;

  const observation_t get_state() const override;

 private:
  void compute_velocities(double F);
  void integrate_internal(double F, double dt);

 private:
  PoleCartDynamicsConfig config_;
  observation_t x_;
  observation_t xd_;
};
}  // namespace pole_cart
