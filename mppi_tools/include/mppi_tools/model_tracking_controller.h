//
// Created by giuseppe on 24.04.21.
//

#pragma once
#include <mppi/controller/mppi.h>
#include "mppi_tools/control_gui.hpp"

//  This class implements a model tracking controller.
//  The same dynamics used for rollouts is used as simulated (controlled)
//  system (no model mismatch)

namespace mppi_tools {

class ModelTrackingController {
 public:
  ModelTrackingController() = default;

  void init(mppi::DynamicsBase::dynamics_ptr dynamics,
            mppi::CostBase::cost_ptr cost, const mppi::observation_t& x0,
            const double& t0, const mppi::SolverConfig& config);
  void get_state(mppi::observation_t& x) const;
  void set_initial_state(const mppi::observation_t& x0, const double& t0);
  void set_reference(mppi::reference_trajectory_t& ref);
  void step();

 public:
  bool initialized_ = false;

  double t_;
  mppi::input_t u_;
  mppi::observation_t x_;

  mppi::DynamicsBase::dynamics_ptr model_;
  std::unique_ptr<mppi::PathIntegral> solver_;

  mppi_tools::ControlGui gui_;
};
}  // namespace mppi_tools
