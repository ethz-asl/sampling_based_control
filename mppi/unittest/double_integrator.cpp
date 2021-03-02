/*!
 * @file     double_integrator.cpp
 * @author   Giuseppe Rizzi
 * @date     23.07.2020
 * @version  1.0
 * @brief    description
 */

#include <Eigen/Dense>
#include <array>
#include <chrono>
#include "mppi/controller/mppi.h"

using namespace Eigen;
using namespace mppi;

// Simple dynamics class representing a point mass subject to a force
class DoubleIntegratorDynamics : public DynamicsBase {
 public:
  DoubleIntegratorDynamics() {
    A << 0.0, 1.0, 0.0, 0.0;
    B << 0.0, 1.0;
    x << 0.0, 0.0;
  };
  ~DoubleIntegratorDynamics() = default;

  size_t get_input_dimension() override { return 1; }
  size_t get_state_dimension() override { return 2; }

  dynamics_ptr create() override { return mppi::DynamicsBase::dynamics_ptr(); }
  dynamics_ptr clone() const override {
    return std::make_shared<DoubleIntegratorDynamics>(*this);
  }

  void reset(const observation_t& xr) override { x = xr; }

  observation_t step(const input_t& u, const double dt) override {
    x += (A * x + B * u) * dt;
    return x;
  }

  // does not make sense, just to test this function is used properly in the
  // control loop
  input_t get_zero_input(const observation_t& x) override {
    return input_t::Ones(1) * x(0);
  }

 private:
  Eigen::Vector2d x;
  Eigen::Matrix<double, 2, 2> A;
  Eigen::Matrix<double, 2, 1> B;
};

// Simple cost class driving the point mass to 1
class DoubleIntegratorCost : public CostBase {
 public:
  DoubleIntegratorCost() = default;
  ~DoubleIntegratorCost() = default;
  cost_ptr create() override { return mppi::CostBase::cost_ptr(); }
  cost_ptr clone() const override { return mppi::CostBase::cost_ptr(); }
  cost_t compute_cost(const observation_t& x, const reference_t& r,
                      const double t) {
    return w * (x(0) - 1.0) * (x(0) - 1.0);
  }

 private:
  double w = 10;
};
