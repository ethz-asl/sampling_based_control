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

namespace panda_mobile {

enum PandaMobileDim {
  STATE_DIMENSION = 10,     // q, x, y, yaw
  INPUT_DIMENSION = 10,     // q_dot, x_dot, y_dot, yaw_dot
  REFERENCE_DIMENSION = 10  // x_t, x_q, obstacle_t
};

class PandaMobileDynamics : public mppi::DynamicsBase {
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

  dynamics_ptr create() override {
    return std::make_shared<PandaMobileDynamics>(robot_description_,
                                                 holonomic_);
  }

  dynamics_ptr clone() const override {
    return std::make_shared<PandaMobileDynamics>(*this);
  }

  void reset(const observation_t& x) override;

  observation_t step(const input_t& u, const double dt) override;
  input_t get_zero_input(const observation_t& x) override;

  const observation_t get_state() const override;

 private:
  observation_t x_;

  std::string robot_description_;

  // holonomic vs non-holonimic base
  bool holonomic_;
};
}  // namespace panda_mobile
