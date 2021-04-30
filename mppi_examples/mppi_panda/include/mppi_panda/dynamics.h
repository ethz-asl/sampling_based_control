/*!
 * @file     pendulumcartdynamics.h
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#pragma once
#include <math.h>
#include <mppi/core/dynamics.h>
#include <ros/package.h>
#include <Eigen/Core>
#include <iostream>
#include <stdexcept>

#include "mppi_panda/dimensions.h"

namespace panda {

class PandaDynamics : public mppi::Dynamics {
 public:
  PandaDynamics(const std::string& robot_description = "")
      : robot_description_(robot_description) {
    x_ = mppi::observation_t::Zero(PandaDim::STATE_DIMENSION);
  };
  ~PandaDynamics() = default;

 public:
  size_t get_input_dimension() override { return PandaDim::INPUT_DIMENSION; }
  size_t get_state_dimension() override { return PandaDim::STATE_DIMENSION; }

  mppi::dynamics_ptr create() override {
    return std::make_shared<PandaDynamics>(robot_description_);
  }

  mppi::dynamics_ptr clone() const override {
    return std::make_shared<PandaDynamics>(*this);
  }

  void reset(const mppi::observation_t& x) override;

  mppi::observation_t step(const mppi::input_t& u, const double dt) override;
  const mppi::observation_t get_state() const override;
  mppi::input_t get_zero_input(const mppi::observation_t& x) override;

 private:
  mppi::observation_t x_;
  std::string robot_description_;
};
}  // namespace panda
