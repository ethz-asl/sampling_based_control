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
#include <ros/package.h>
#include <Eigen/Core>
#include <iostream>
#include <stdexcept>

#include "mppi_panda/dimensions.h"

namespace panda {

class PandaDynamics : public mppi::DynamicsBase {
 public:
  PandaDynamics(const std::string& robot_description = "")
      : robot_description_(robot_description) {
    x_ = observation_t::Zero(PandaDim::STATE_DIMENSION);
  };
  ~PandaDynamics() = default;

 public:
  size_t get_input_dimension() override { return PandaDim::INPUT_DIMENSION; }
  size_t get_state_dimension() override { return PandaDim::STATE_DIMENSION; }

  dynamics_ptr create() override {
    return std::make_shared<PandaDynamics>(robot_description_);
  }

  dynamics_ptr clone() const override {
    return std::make_shared<PandaDynamics>(*this);
  }

  void reset(const observation_t& x) override;

  observation_t step(const input_t& u, const double dt) override;
  const observation_t get_state() const override;
  input_t get_zero_input(const observation_t& x) override;

 private:
  observation_t x_;
  std::string robot_description_;
};
}  // namespace panda
