/*!
 * @file     mppi_new.cpp
 * @author   Giuseppe Rizzi
 * @date     01.07.2020
 * @version  1.0
 * @brief    description
 */

#include <algorithm>

#include <Eigen/Core>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <vector>

#include "mppi/core/config.h"
#include "mppi/core/cost.h"
#include "mppi/core/dynamics.h"
#include "mppi/core/rollout.h"
#include "mppi/core/solver.h"

#include "mppi/utils/logging.h"
#include "mppi/utils/savgol_filter.h"

#ifdef SIGNAL_LOGGER
#include <signal_logger/signal_logger.hpp>
#endif

namespace mppi {

Solver::Solver(dynamics_ptr dynamics, cost_ptr cost, policy_ptr policy,
               const Config& config)
    : dynamics_(std::move(dynamics)),
      cost_(std::move(cost)),
      policy_(std::move(policy)),
      config_(config) {
  init_data();
  init_threading();

  if (config_.display_update_freq) {
    start_time_ = std::chrono::high_resolution_clock::now();
  }

#ifdef SIGNAL_LOGGER
  signal_logger::add(min_cost_, "solver/rollouts/min_cost");
  signal_logger::add(max_cost_, "solver/rollouts/max_cost");
  signal_logger::add(rollouts_cost_, "solver/rollouts/costs");
  signal_logger::add(delay_steps_, "solver/delay_steps");
  signal_logger::add(rate_, "solver/rate");
  signal_logger::logger->updateLogger();
#endif
}

void Solver::init_data() {
  // TODO(giuseppe) this should be automatically computed when config is parsed
  steps_ = static_cast<int>(std::ceil(config_.horizon / config_.step_size));
  nx_ = dynamics_->get_state_dimension();
  nu_ = dynamics_->get_input_dimension();

  opt_roll_ = Rollout(steps_, nu_, nx_);
  opt_roll_cache_ = Rollout(steps_, nu_, nx_);
  nominal_.setZero(steps_, nu_);

  weights_.resize(config_.rollouts, 1.0 / config_.rollouts);
  rollouts_.resize(config_.rollouts, Rollout(steps_, nu_, nx_));
  cached_rollouts_ = std::ceil(config_.caching_factor * config_.rollouts);

  delay_steps_ = 0;
  step_count_ = 0;
  observation_set_ = false;
  reference_set_ = false;
}

void Solver::init_filter() {}

void Solver::init_threading() {
  if (config_.threads > 1) {
    std::cout << "Using multithreading. Number of threads: " << config_.threads
              << std::endl;
    pool_ = std::make_unique<ThreadPool>(config_.threads);
    futures_.resize(config_.threads);

    for (size_t i = 0; i < config_.threads; i++) {
      dynamics_v_.push_back(dynamics_->create());
      cost_v_.push_back(cost_->create());
    }
  }
}

void Solver::update_policy() {
  if (!observation_set_) {
    log_warning_throttle(1.0,
                         "Observation has never been set. Dropping update");
  } else if (!reference_set_) {
    log_warning_throttle(1.0, "Reference has never been set. Dropping update");
  } else {
    auto start = std::chrono::steady_clock::now();  
    update_delay();
    copy_observation();

    for (size_t i = 0; i < config_.substeps; i++) {
      prepare_rollouts();
      update_reference();      
      sample_trajectories();
      optimize();
      filter_input();

      stage_cost_ =
          cost_->get_stage_cost(x0_internal_, opt_roll_.uu[0], t0_internal_);
    }
    swap_policies();
    auto end = std::chrono::steady_clock::now();
    rate_ = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1e9;
  }
}

void Solver::time_it() {
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start_time_)
          .count();
  auto timing_string =
      "Time since optimization start: " + std::to_string(duration) + "[μs], " +
      std::to_string(duration / 1000000.0) + "[s], " +
      std::to_string(1 / (duration / 1000000.0)) + "[Hz]";
  log_info(timing_string);
  start_time_ = std::chrono::high_resolution_clock::now();
}

void Solver::set_observation(const observation_t& x, const double t) {
  {
    std::unique_lock<std::shared_mutex> lock(state_mutex_);
    x0_ = x;
    reset_time_ = t;
  }

  // initialization of rollouts data
  if (first_step_) {
    copy_observation();
    initialize_rollouts();
    first_step_ = false;
  }

  observation_set_ = true;
}

void Solver::update_delay() {
  delay_steps_ = std::ceil((reset_time_ - t0_internal_) / config_.step_size);
  policy_->update_delay(delay_steps_);
}

void Solver::copy_observation() {
  std::shared_lock<std::shared_mutex> lock(state_mutex_);
  x0_internal_ = x0_;
  t0_internal_ = reset_time_;
}

void Solver::initialize_rollouts() {
  std::shared_lock<std::shared_mutex> lock_state(state_mutex_);
  opt_roll_.clear();
  std::fill(opt_roll_.uu.begin(), opt_roll_.uu.end(),
            dynamics_->get_zero_input(x0_internal_));

  std::shared_lock<std::shared_mutex> lock(rollout_cache_mutex_);
  std::fill(opt_roll_cache_.xx.begin(), opt_roll_cache_.xx.end(), x0_internal_);
  std::fill(opt_roll_cache_.uu.begin(), opt_roll_cache_.uu.end(),
            dynamics_->get_zero_input(x0_internal_));
  for (int i = 0; i < steps_; i++) {
    opt_roll_cache_.tt[i] = t0_internal_ + config_.step_size * i;
  }
}

void Solver::prepare_rollouts() {
  // cleanup
  rollouts_cost_.clear();
  for (auto& roll : rollouts_) {
    roll.clear_cost();
    roll.valid = true;
  }
}

void Solver::set_reference_trajectory(mppi::reference_trajectory_t& ref) {
  if (ref.rr.size() != ref.tt.size()) {
    std::stringstream error;
    error << "The reference trajectory state and time dimensions do not match: "
          << ref.rr.size() << " != " << ref.tt.size();
    throw std::runtime_error(error.str());
  }
  std::unique_lock<std::shared_mutex> lock(reference_mutex_);
  rr_tt_ref_ = ref;
  reference_set_ = true;
}

void Solver::update_reference() {
  std::shared_lock<std::shared_mutex> lock(reference_mutex_);
  cost_->set_reference_trajectory(rr_tt_ref_);

  if (config_.threads > 1) {
    for (auto& cost : cost_v_) cost->set_reference_trajectory(rr_tt_ref_);
  }

}

void Solver::sample_noise(input_t& noise) { sampler_->get_sample(noise); }

void Solver::sample_trajectories_batch(dynamics_ptr& dynamics, cost_ptr& cost,
                                       const size_t start_idx,
                                       const size_t end_idx) {
  observation_t x;
  for (size_t k = start_idx; k < end_idx; k++) {
    dynamics->reset(x0_internal_, t0_internal_);
    x = x0_internal_;
    double ts;
    for (int t = 0; t < steps_; t++) {
      ts = t0_internal_ + t * config_.step_size;
      rollouts_[k].tt[t] = ts;
      rollouts_[k].uu[t] = policy_->sample(ts, k);

      // compute input-state stage cost
      double cost_temp;
      cost_temp = std::pow(config_.discount_factor, t) *
                  cost->get_stage_cost(x, rollouts_[k].uu[t], ts);

      if (std::isnan(cost_temp)) {
        std::stringstream ss;
        ss << "Something went wrong ... dynamics diverged?\n" << std::endl;
        ss << "Rollout#" << k << std::endl;
        ss << "Time  where diverged: " << t << std::endl;
        ss << "State where diverged: " << x.transpose() << std::endl;
        ss << "Previous state: " << rollouts_[k].xx[t - 1].transpose()
           << std::endl;
        ss << "Input where diverged: " << rollouts_[k].uu[t].transpose();
        ss << "Previous input: " << rollouts_[k].uu[t - 1].transpose()
           << std::endl;
        rollouts_[k].valid = false;
        rollouts_[k].total_cost = std::numeric_limits<double>::infinity();
        log_warning(ss.str());
        break;
      }

      // store data
      rollouts_[k].xx[t] = x;
      rollouts_[k].cc(t) = cost_temp;
      rollouts_[k].total_cost += cost_temp;

      // integrate dynamics    auto start = std::chrono::steady_clock::now();
      x = dynamics->step(rollouts_[k].uu[t], config_.step_size);
    }
  }
}

void Solver::sample_trajectories() {
  policy_->shift(t0_internal_);
  policy_->update_samples(weights_, cached_rollouts_);

  if (config_.threads == 1) {
    sample_trajectories_batch(dynamics_, cost_, 0, config_.rollouts);
  } else {
    for (size_t i = 0; i < config_.threads; i++) {
      futures_[i] = pool_->enqueue(
          std::bind(&Solver::sample_trajectories_batch, this,
                    std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3, std::placeholders::_4),
          dynamics_v_[i], cost_v_[i],
          (size_t)i * config_.rollouts / config_.threads,
          (size_t)(i + 1) * config_.rollouts / config_.threads);
    }

    for (size_t i = 0; i < config_.threads; i++) futures_[i].get();
  }
}

void Solver::compute_weights() {
  // keep all non diverged rollouts
  double min_cost = std::numeric_limits<double>::max();
  double max_cost = -min_cost;

  for (size_t k = 0; k < config_.rollouts; k++) {
    if (rollouts_[k].valid) {
      const double& cost = rollouts_[k].total_cost;
      min_cost = (cost < min_cost) ? cost : min_cost;
      max_cost = (cost > max_cost) ? cost : max_cost;
    }
  }

  min_cost_ = min_cost;   //*std::min_element(rollouts_cost_.begin(), rollouts_cost_.end());
  max_cost_ = max_cost;   //*std::max_element(rollouts_cost_.begin(), rollouts_cost_.end());

  double sum = 0.0;
  for (size_t k = 0; k < config_.rollouts; k++) {
    double modified_cost = config_.h * (rollouts_[k].total_cost - min_cost_) /
                           (max_cost_ - min_cost_);

    weights_[k] = rollouts_[k].valid ? std::exp(-modified_cost) : 0.0;
    sum += weights_[k];
  }
  std::transform(weights_.begin(), weights_.end(), weights_.begin(),
                 [&sum](double v) -> double { return v / sum; });
}

void Solver::optimize() {
  // get new rollouts weights
  compute_weights();

  // update policy according to new weights
  policy_->update(weights_, config_.alpha);

  // retrieve the nominal policy for each time step
  for (int t = 0; t < steps_; t++) {
    opt_roll_.tt[t] = t0_internal_ + t * config_.step_size;
    opt_roll_.uu[t] = policy_->nominal(t0_internal_ + t * config_.step_size);
  }

}

void Solver::filter_input() {
  // only if filter is available reset to the initial opt time
  if (filter_) {
    filter_->reset(x0_internal_, t0_internal_);
  }

  // reset the dynamics such that we rollout the dynamics again
  // with the input we filter step after step (sequentially)
  dynamics_->reset(x0_internal_, t0_internal_);
  opt_roll_.xx[0] = x0_internal_;

  // sequential filtering otherwise just nominal rollout
  for (int t = 0; t < steps_ - 1; t++) {
    if (filter_) {
      filter_->apply(opt_roll_.xx[t], opt_roll_.uu[t], opt_roll_.tt[t]);
      nominal_.row(t) = opt_roll_.uu[t].transpose();
    }
    opt_roll_.xx[t + 1] = dynamics_->step(opt_roll_.uu[t], config_.step_size);
  }

  // filter the last input in the sequence (if filter is available)
  if (filter_) {
    filter_->apply(opt_roll_.xx.back(), opt_roll_.uu.back(),
                   opt_roll_.tt.back());
    nominal_.bottomRows(1) = opt_roll_.uu.back().transpose();
    policy_->set_nominal(nominal_);
  }
}

void Solver::get_input(const observation_t& x, input_t& u, const double t) {
  static double coeff;
  static size_t idx;
  {
    std::shared_lock<std::shared_mutex> lock(rollout_cache_mutex_);
    if (t < opt_roll_cache_.tt.front()) {
      std::stringstream warning;
      warning << "Queried time " << t << " smaller than first available time "
              << opt_roll_cache_.tt.front();
      log_warning_throttle(1.0, warning.str());
      u = dynamics_->get_zero_input(x);
    }

    auto lower = std::lower_bound(opt_roll_cache_.tt.begin(),
                                  opt_roll_cache_.tt.end(), t);
    if (lower == opt_roll_cache_.tt.end()) {
      std::stringstream warning;
      warning << "Queried time " << t << " larger than last available time "
              << opt_roll_cache_.tt.back();
      log_warning_throttle(1.0, warning.str());
      u = opt_roll_cache_.uu.back();
      return;
    }

    idx = std::distance(opt_roll_cache_.tt.begin(), lower);
    // first index (time)
    if (idx == 0) {
      u = opt_roll_cache_.uu.front();
    }
    // last index (time larget then last step)
    else if (idx > opt_roll_cache_.steps_) {
      u = opt_roll_cache_.uu.back();
    }
    // interpolate
    else {
      coeff = (t - *(lower - 1)) / (*lower - *(lower - 1));
      u = (1 - coeff) * opt_roll_cache_.uu[idx - 1] +
          coeff * opt_roll_cache_.uu[idx];
    }
  }
}

void Solver::get_diagonal_variance(Eigen::VectorXd& var) const {
  var = sampler_->sigma().diagonal();
}

void Solver::get_input_state(const observation_t& x, observation_t& x_nom,
                             input_t& u_nom, const double t) {
  {
    std::shared_lock<std::shared_mutex> lock(rollout_cache_mutex_);
    if (t < opt_roll_cache_.tt.front()) {
      std::stringstream warning;
      warning << "Queried time " << t << " smaller than first available time "
              << opt_roll_cache_.tt.front();
      log_warning_throttle(1.0, warning.str());
      x_nom = opt_roll_cache_.xx.front();
      u_nom = dynamics_->get_zero_input(x);
      return;
    }

    auto lower = std::lower_bound(opt_roll_cache_.tt.begin(),
                                  opt_roll_cache_.tt.end(), t);
    if (lower == opt_roll_cache_.tt.end()) {
      std::stringstream warning;
      warning << "Queried time " << t << " larger than last available time "
              << opt_roll_cache_.tt.back();
      log_warning_throttle(1.0, warning.str());
      x_nom = opt_roll_cache_.xx.back();
      u_nom = dynamics_->get_zero_input(x);
      return;
    }

    size_t idx = std::distance(opt_roll_cache_.tt.begin(), lower);

    // first
    if (idx == 0) {
      x_nom = opt_roll_cache_.xx.front();
      u_nom = opt_roll_cache_.uu.front();
    }
    // last
    else if (idx > opt_roll_cache_.steps_) {
      x_nom = opt_roll_cache_.xx.back();
      u_nom = opt_roll_cache_.uu.back();
    }
    // interpolate
    else {
      double coeff = (t - *(lower - 1)) / (*lower - *(lower - 1));
      u_nom = (1 - coeff) * opt_roll_cache_.uu[idx - 1] +
              coeff * opt_roll_cache_.uu[idx];
      x_nom = opt_roll_cache_.xx[idx];  // TODO offer a way to also do
                                        // interpolation of the state
    }
  }
}

bool Solver::get_optimal_rollout(observation_array_t& xx, input_array_t& uu) {
  std::shared_lock<std::shared_mutex> lock(rollout_cache_mutex_);
  auto lower = std::lower_bound(opt_roll_cache_.tt.begin(),
                                opt_roll_cache_.tt.end(), reset_time_);
  if (lower == opt_roll_cache_.tt.end()) return false;
  size_t offset = std::distance(opt_roll_cache_.tt.begin(), lower);

  // fill with portion of vector starting from current time
  xx = observation_array_t(opt_roll_cache_.xx.begin() + offset,
                           opt_roll_cache_.xx.end());
  uu = input_array_t(opt_roll_cache_.uu.begin() + offset,
                     opt_roll_cache_.uu.end());
  return true;
}

void Solver::get_optimal_rollout(Rollout& r) {
  std::shared_lock<std::shared_mutex> lock(rollout_cache_mutex_);
  
  // fill with portion of vector starting from current reset time
  r.tt = time_array_t(opt_roll_cache_.tt.begin(),
                      opt_roll_cache_.tt.end());

  r.xx = observation_array_t(opt_roll_cache_.xx.begin(),
                             opt_roll_cache_.xx.end());
  
  r.uu = input_array_t(opt_roll_cache_.uu.begin(),
                       opt_roll_cache_.uu.end());
}

void Solver::swap_policies() {
  std::unique_lock<std::shared_mutex> lock(rollout_cache_mutex_);
  opt_roll_cache_ = opt_roll_;
}

template <typename T>
void Solver::shift_back(std::vector<T>& v_out, const T& fill,
                        const int offset) {
  std::rotate(v_out.begin(), v_out.begin() + offset, v_out.end());
  std::fill(v_out.end() - offset, v_out.end(), fill);
}

// explicit instantiation
template void Solver::shift_back<Eigen::VectorXd>(
    std::vector<Eigen::VectorXd>& v_out, const Eigen::VectorXd& fill,
    const int offset);

void Solver::print_cost_histogram() const {
  constexpr int nbins = 10;
  double delta = (max_cost_ - min_cost_) / nbins;

  std::cout << "Rollouts cost histogram " << std::endl;
  std::cout << "Max: " << max_cost_ << ", Min: " << min_cost_
            << ", delta: " << delta << std::endl;
  std::map<int, int> hist{};
  for (const auto& cost : rollouts_cost_) {
    ++hist[std::round((cost - min_cost_) / delta)];
  }
  for (auto p : hist) {
    std::cout << std::setw(2) << p.first << ' ' << std::string(p.second, '*')
              << '\n';
  }
  std::cout << std::endl << std::endl;
}

}  // namespace mppi
