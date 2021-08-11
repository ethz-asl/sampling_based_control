/*!
 * @file     pendulumcartdynamics.h
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#pragma once
#include <mppi/core/dynamics.h>
#include <ros/package.h>
#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace panda_mobile {

enum PandaMobileDim {
  STATE_DIMENSION = 10,     // q, x, y, yaw
  INPUT_DIMENSION = 10,     // q_dot, x_dot, y_dot, yaw_dot
  REFERENCE_DIMENSION = 10  // x_t, x_q, obstacle_t
};

class PandaMobileDynamics : public mppi::Dynamics {
 public:
  PandaMobileDynamics(const std::string& robot_description,
                      bool holonomic = true);
  ~PandaMobileDynamics() = default;

 public:
  size_t get_input_dimension() override {
    return PandaMobileDim::INPUT_DIMENSION;
  }
  size_t get_state_dimension() override {
    return PandaMobileDim::STATE_DIMENSION;
  }

  mppi::dynamics_ptr create() override {
    return std::make_shared<PandaMobileDynamics>(robot_description_,
                                                 holonomic_);
  }

  mppi::dynamics_ptr clone() const override {
    return std::make_shared<PandaMobileDynamics>(*this);
  }

  void reset(const mppi::observation_t& x, const double t) override;

  mppi::observation_t step(const mppi::input_t& u, const double dt) override;
  mppi::input_t get_zero_input(const mppi::observation_t& x) override;

  const mppi::observation_t get_state() const override;

 private:
  mppi::observation_t x_;

  bool holonomic_;
  std::string robot_description_;
};
}  // namespace panda_mobile
