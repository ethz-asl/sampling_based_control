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
  ModelTrackingController(mppi::DynamicsBase::dynamics_ptr dynamics,
                          mppi::CostBase::cost_ptr cost,
                          const mppi::SolverConfig& config);

 private:
  std::unique_ptr<mppi::PathIntegral> solver_;
  mppi_tools::ControlGui gui_;
};
}  // namespace mppi_tools
