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

namespace fom {

enum FOMDim {
  STATE_DIMENSION = 2,     // x, y
  INPUT_DIMENSION = 2,     // xd, yd
  REFERENCE_DIMENSION = 2  // x_goal, y_goal
};

class FOMDynamics : public mppi::Dynamics {
 public:
  explicit FOMDynamics(const double dt) {
    x_ = mppi::observation_t::Zero(FOMDim::STATE_DIMENSION);
    dt_ = dt;
  };
  ~FOMDynamics() = default;

 public:
  size_t get_input_dimension() override { return FOMDim::INPUT_DIMENSION; }
  size_t get_state_dimension() override { return FOMDim::STATE_DIMENSION; }

  mppi::dynamics_ptr create() override {
    return std::make_shared<FOMDynamics>(dt_);
  }

  mppi::dynamics_ptr clone() const override {
    return std::make_shared<FOMDynamics>(*this);
  }

  void reset(const mppi::observation_t& x) override;

  mppi::observation_t step(const mppi::input_t& u, const double dt) override;

  const mppi::observation_t get_state() const override;

 private:
  double dt_;
  mppi::observation_t x_;
};
}  // namespace fom
