//
// Created by giuseppe on 24.04.21.
//

#include "mppi_tools/model_tracking_controller.h"

using namespace std::chrono;

using namespace mppi;
using namespace mppi_tools;
using std::placeholders::_1;

void ModelTrackingController::init(mppi::dynamics_ptr dynamics,
                                   mppi::cost_ptr cost,
                                   const mppi::observation_t &x0,
                                   const double &t0,
                                   const mppi::config_t &config) {
  solver_ = std::make_unique<mppi::Solver>(dynamics, cost, config);
  model_ = dynamics->create();
  set_initial_state(x0, t0);

  gui_.init();
  param_options_t<float> control_rate_opt;
  control_rate_opt.value = 100.0;
  control_rate_opt.default_value = 100.0;
  control_rate_opt.upper = 200.0;
  control_rate_opt.lower = 10.0;
  control_rate_opt.callback =
      std::bind(&ModelTrackingController::set_control_rate, this, _1);
  gui_.register_param("control rate", control_rate_opt);

  initialized_ = true;

  mppi::WorkerOptions sim_thread_opt;
  sim_thread_opt.name_ = "simulation_thread";
  sim_thread_opt.timeStep_ = model_->get_dt();
  sim_thread_opt.callback_ =
      std::bind(&ModelTrackingController::run_sim, this, std::placeholders::_1);
  worker_manager_.addWorker(sim_thread_opt, false);

  mppi::WorkerOptions control_thread_opt;
  control_thread_opt.name_ = "control_thread";
  control_thread_opt.timeStep_ = 0.01;
  control_thread_opt.callback_ = std::bind(
      &ModelTrackingController::run_control, this, std::placeholders::_1);
  worker_manager_.addWorker(control_thread_opt, false);

  mppi::WorkerOptions gui_thread_opt;
  gui_thread_opt.name_ = "gui_thread";
  gui_thread_opt.timeStep_ = 1.0 / 30.0;
  gui_thread_opt.callback_ = std::bind(&ModelTrackingController::update_gui,
                                       this, std::placeholders::_1);
  worker_manager_.addWorker(gui_thread_opt, true);
}

bool ModelTrackingController::set_control_rate(const double &rate) {
  worker_manager_.setWorkerTimestep("control_thread", 1.0 / rate);
  return true;
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

bool ModelTrackingController::run_sim(const mppi::WorkerEvent &event) {
  if (!gui_.should_pause()) {
    std::unique_lock<std::mutex> input_lock(input_mutex_);
    std::unique_lock<std::mutex> state_lock(state_mutex_);

    solver_->get_input(x_, u_, t_);
    x_ = model_->step(u_, solver_->config_.step_size);
    t_ += solver_->config_.step_size;
  }

  return true;
}

bool ModelTrackingController::run_control(const mppi::WorkerEvent &event) {
  static std::chrono::time_point<std::chrono::steady_clock> start, end;
  static double elapsed;

  if (!gui_.should_pause()) {
    {
      std::unique_lock<std::mutex> lock(state_mutex_);
      solver_->set_observation(x_, t_);
    }
    start = std::chrono::steady_clock::now();
    solver_->update_policy();
    end = std::chrono::steady_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start)
                  .count() /
              1.0e9;
    {
      std::unique_lock<std::mutex> lock(gui_mutex_);
      gui_.reset_optimization_time(elapsed);
      gui_.reset_rollouts(solver_->rollouts_);
      gui_.reset_policy(solver_->get_optimal_rollout().uu);
      gui_.reset_weights(solver_->get_weights());
    }
  }
  return true;
}

bool ModelTrackingController::update_gui(const mppi::WorkerEvent &event) {
  std::unique_lock<std::mutex> lock(gui_mutex_);
  gui_.render();
  return true;
}

void ModelTrackingController::step() {
  if (!initialized_) return;

  if (gui_.should_start() && !started_) {
    worker_manager_.startWorkers();
    started_ = true;
  }
  {
//    std::unique_lock<std::mutex> lock(gui_mutex_);
//    gui_.render();
  }
}