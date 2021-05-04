//
// Created by giuseppe on 24.04.21.
//

#pragma once
#include <mppi/core/solver.h>
#include <mppi/utils/timer.h>
#include <mppi/threading/WorkerManager.hpp>

#include "mppi_tools/control_gui.hpp"

//  This class implements a model tracking controller.
//  The same dynamics used for rollouts is used as simulated (controlled)
//  system (no model mismatch)

namespace mppi_tools {

class ModelTrackingController {
 public:
  ModelTrackingController() = default;

  void init(mppi::dynamics_ptr dynamics, mppi::cost_ptr cost,
            const mppi::observation_t& x0, const double& t0,
            const mppi::config_t& config);
  void get_state(mppi::observation_t& x) const;
  void set_initial_state(const mppi::observation_t& x0, const double& t0);
  void set_reference(mppi::reference_trajectory_t& ref);
  void step();

 private:
  bool update_gui(const mppi::WorkerEvent& );
  bool run_sim(const mppi::WorkerEvent& );
  bool run_control(const mppi::WorkerEvent& );

  bool set_control_rate(const double& );

 public:
  bool initialized_ = false;
  bool started_ = false;

  double t_;
  std::mutex input_mutex_;
  mppi::input_t u_;

  std::mutex state_mutex_;
  mppi::observation_t x_;

  std::mutex gui_mutex_;
  mppi_tools::ControlGui gui_;

  mppi::dynamics_ptr model_;  // simulator
  mppi::solver_ptr solver_;   // controller

  mppi::Timer timer_;
  mppi::WorkerManager worker_manager_;
};
}  // namespace mppi_tools
