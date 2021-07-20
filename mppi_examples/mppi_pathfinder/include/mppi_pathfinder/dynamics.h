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

namespace pathfinder {

enum PathfinderDim {
  STATE_DIMENSION = 3,    // x, y, theta, v
  INPUT_DIMENSION = 1,    // acceleration
  REFERENCE_DIMENSION = 1 // position
};

struct PathfinderDynamicsConfig {
  double mc = 1.0;
  double tau_theta = 0.7;
  double dt_internal = 0.001;
};

class PathfinderDynamics : public mppi::DynamicsBase {
 public:
  PathfinderDynamics() {
    x_ = observation_t::Zero(PathfinderDim::STATE_DIMENSION);
    xd_ = observation_t::Zero(PathfinderDim::STATE_DIMENSION);
  };

  explicit PathfinderDynamics(const PathfinderDynamicsConfig& config)
      : PathfinderDynamics() {
    config_ = config;
  };

  ~PathfinderDynamics() = default;

 public:
  void set_dynamic_properties(const PathfinderDynamicsConfig& config) {
    config_ = config;
  }

  size_t get_input_dimension() override { return PathfinderDim::INPUT_DIMENSION; }
  size_t get_state_dimension() override { return PathfinderDim::STATE_DIMENSION; }

  dynamics_ptr create() override {
    return std::make_shared<PathfinderDynamics>();
  }

  dynamics_ptr clone() const override {
    return std::make_shared<PathfinderDynamics>(*this);
  }

  void reset(const observation_t& x) override;

  observation_t step(const input_t& u, const double dt) override;

  const observation_t get_state() const override;

 private:
   void compute_velocities(double a);
   void integrate_internal(double u, double dt);

 private:
  PathfinderDynamicsConfig config_;
  observation_t x_;
  observation_t xd_;
};
}  // namespace pathfinder
