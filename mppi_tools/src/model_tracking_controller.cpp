//
// Created by giuseppe on 24.04.21.
//

#include "mppi_tools/model_tracking_controller.h"

using namespace mppi;
using namespace mppi_tools;

void ModelTrackingController::init(mppi::dynamics_ptr dynamics,
                                   mppi::cost_ptr cost,
                                   mppi::policy_ptr policy,
                                   const mppi::observation_t &x0,
                                   const double &t0,
                                   const mppi::config_t &config) {
  solver_ = std::make_unique<mppi::Solver>(dynamics, cost, policy, config);
  model_ = dynamics->clone();
  set_initial_state(x0, t0);

  gui_.init();
  initialized_ = true;
}

void ModelTrackingController::get_state(mppi::observation_t &x) const {
  x = x_;
}

void ModelTrackingController::set_initial_state(const mppi::observation_t &x0,
                                                const double &t0) {
  t_ = t0;
  x_ = x0;
  model_->reset(x0);
  solver_->set_observation(x0, t_);
}

void ModelTrackingController::set_reference(mppi::reference_trajectory_t &ref) {
  if (initialized_) solver_->set_reference_trajectory(ref);
}

void ModelTrackingController::step() {
  if (!initialized_) return;

  // sim + control
  if (gui_.should_pause()) {
    if (gui_.step_simulation()) {
      solver_->set_observation(x_, t_);
      solver_->get_input(x_, u_, t_);
      x_ = model_->step(u_, solver_->config_.step_size);
      t_ += solver_->config_.step_size;
      gui_.step_simulation() = false;
    }
    if (gui_.step_controller()) {
      solver_->update_policy();
      gui_.step_controller() = false;
    }
    if (gui_.step_all()) {
      solver_->set_observation(x_, t_);
      solver_->update_policy();
      solver_->get_input(x_, u_, t_);
      x_ = model_->step(u_, solver_->config_.step_size);
      t_ += solver_->config_.step_size;
      gui_.step_all() = false;
    }
  } else {
    solver_->set_observation(x_, t_);
    solver_->update_policy();
    solver_->get_input(x_, u_, t_);
    x_ = model_->step(u_, solver_->config_.step_size);
    t_ += solver_->config_.step_size;
  }

  // gui
  gui_.reset_rollouts(solver_->rollouts_);
  gui_.reset_policy(solver_->get_optimal_rollout().uu);
  // gui_.reset_averaged_policy(solver_->); no averaged policy available in the
  // controller
  gui_.reset_weights(solver_->get_weights());
  gui_.render();
}