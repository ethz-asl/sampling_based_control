//
// Created by giuseppe on 24.04.21.
//

#include "mppi_tools/model_tracking_controller.h"

using namespace mppi;
using namespace mppi_tools;

ModelTrackingController::ModelTrackingController(
    DynamicsBase::dynamics_ptr dynamics, CostBase::cost_ptr cost,
    const SolverConfig &config) {
  solver_ = std::make_unique<PathIntegral>(dynamics, cost, config);
  model_ = dynamics->clone();
  t_ = 0;
}

void ModelTrackingController::get_state(mppi::observation_t &x) const {
  x = x_;
}

void ModelTrackingController::set_initial_state(const mppi::observation_t &x0) {
  t_ = 0.0;
  x_ = x0;
  model_->reset(x0);
  solver_->set_observation(x0, t_);
}

void ModelTrackingController::set_reference(mppi::reference_trajectory_t &ref) {
  solver_->set_reference_trajectory(ref);
}

void ModelTrackingController::run() {
  solver_->set_observation(x_, t_);
  solver_->update_policy();
  solver_->get_input(x_, u_, t_);
  x_ = model_->step(u_, t_);
  t_ += model_->get_dt();

  gui_.reset_rollouts(solver_->rollouts_);
  gui_.reset_policy(solver_->get_optimal_rollout().uu);
  // gui_.reset_averaged_policy(solver_->); no averaged policy available in the
  // controller
  gui_.reset_weights(solver_->get_weights());
}