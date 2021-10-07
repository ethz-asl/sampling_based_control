//
// Created by giuseppe on 24.04.21.
//

#pragma once
#include <mppi/core/solver.h>
#include "mppi_tools/control_gui.hpp"

//  This class implements a model tracking controller.
//  The same dynamics used for rollouts is used as simulated (controlled)
//  system (no model mismatch)

namespace mppi_tools {

class ModelTrackingController {
 public:
  ModelTrackingController() = default;

  void init(mppi::dynamics_ptr dynamics, mppi::cost_ptr cost,
            mppi::policy_ptr policy,
            const mppi::observation_t& x0, const double& t0,
            const mppi::config_t& config);
  void get_state(mppi::observation_t& x) const;
  void set_initial_state(const mppi::observation_t& x0, const double& t0);
  void set_reference(mppi::reference_trajectory_t& ref);
  void step();

 public:
  bool initialized_ = false;

  double t_;
  mppi::input_t u_;
  mppi::observation_t x_;

  mppi::dynamics_ptr model_;
  mppi::solver_ptr solver_;

  mppi_tools::ControlGui gui_;
};
}  // namespace mppi_tools
