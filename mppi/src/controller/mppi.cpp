/*!
 * @file     mppi_new.cpp
 * @author   Giuseppe Rizzi
 * @date     01.07.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi/controller/mppi.h"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <vector>

#include <Eigen/Core>

#include "mppi/controller/rollout.h"
#include "mppi/cost/cost_base.h"
#include "mppi/dynamics/dynamics_base.h"
#include "mppi/experts/expert.h"
#include "mppi/filters/savgol_filter.h"
#include "mppi/solver_config.h"
#include "mppi/tree/tree_manager.h"
#include "mppi/utils/logging.h"

namespace mppi {

PathIntegral::PathIntegral(dynamics_ptr dynamics, cost_ptr cost,
                           const SolverConfig& config, sampler_ptr sampler,
                           renderer_ptr renderer)
    : cost_(std::move(cost)),
      dynamics_(std::move(dynamics)),
      config_(config),
      sampler_(std::move(sampler)),
      renderer_(std::move(renderer)),
      expert_(config_, dynamics_),
      tree_manager_(dynamics_, cost_, sampler_, config_, &expert_) {
  init_data();
  init_filter();
  if (!config_.use_tree_search) {
    init_threading();
  }

  if (config_.use_tree_search) {
    init_tree_manager_dynamics();
  }

  if (config_.display_update_freq) {
    start_time_ = std::chrono::high_resolution_clock::now();
  }
}

void PathIntegral::init_data() {
  steps_ = std::floor(config_.horizon / config_.step_size);
  nx_ = dynamics_->get_state_dimension();
  nu_ = dynamics_->get_input_dimension();

  opt_roll_ = Rollout(steps_, nu_, nx_);
  opt_roll_cache_ = Rollout(steps_, nu_, nx_);

  rollouts_.resize(config_.rollouts, Rollout(steps_, nu_, nx_));
  rollouts_cost_ = Eigen::ArrayXd::Zero(config_.rollouts);

  omega = Eigen::ArrayXd::Zero(config_.rollouts);
  cached_rollouts_ = std::ceil(config_.caching_factor * config_.rollouts);

  momentum_.resize(steps_, input_t::Zero(nu_));

  if (sampler_ == nullptr) {
    log_info("No sampler provided, using gaussian sampler");
    if (config_.input_variance.size() != nu_)
      throw std::runtime_error(
          "The input variance size is different from the input size");

    sampler_ = std::make_shared<mppi::GaussianSampler>(nu_);
    sampler_->set_covariance(config_.input_variance);
  }

  if (config_.logging) {
    data_.config = config_;
  }

  if (config_.bound_input) {
    if (config_.u_min.size() != nu_) {
      std::stringstream error;
      error << "Bounding input and min constraint size " << config_.u_min.size()
            << " != " << nu_;
      throw std::runtime_error(error.str());
    }
    if (config_.u_max.size() != nu_) {
      std::stringstream error;
      error << "Bounding input and max constraint size " << config_.u_max.size()
            << " != " << nu_;
      throw std::runtime_error(error.str());
    }
  }
  step_count_ = 0;
  observation_set_ = false;
  reference_set_ = false;
}

void PathIntegral::init_filter() {
  switch (config_.filter_type) {
    case InputFilterType::NONE: {
      break;
    }
    case InputFilterType::SAVITZKY_GOLEY: {
      filter_ = SavGolFilter(steps_, nu_, config_.filters_window,
                             config_.filters_order);
      break;
    }
    default: {
      throw std::runtime_error("Unrecognized filter type.");
    }
  }
}

void PathIntegral::init_threading() {
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

void PathIntegral::init_tree_manager_dynamics() {
  for (size_t i = 0; i < config_.rollouts; i++) {
    tree_dynamics_v_.push_back(dynamics_->create());
  }
}

void PathIntegral::update_policy() {
  if (!observation_set_) {
    log_warning_throttle(1.0,
                         "Observation has never been set. Dropping update");
  } else if (!reference_set_) {
    log_warning_throttle(1.0, "Reference has never been set. Dropping update");
  } else {
    copy_observation();

    for (size_t i = 0; i < config_.substeps; i++) {
      prepare_rollouts();
      update_reference();

      if (config_.use_tree_search) {
        timer_.reset();
        update_experts();
        timer_.add_interval("update_experts");
        sample_trajectories_via_tree();
        timer_.add_interval("sample_trajectories_via_tree");
        overwrite_rollouts();
        timer_.add_interval("overwrite_rollouts");
        timer_.print_intervals(100);
      } else {
        sample_trajectories();
      }

      optimize();
      filter_input();

      // TODO move this away. This goes year since there might be filtering
      // happening before optimal rollout
      dynamics_->reset(x0_internal_);
      for (size_t t = 0; t < steps_; t++) {
        opt_roll_.xx[t] = dynamics_->step(opt_roll_.uu[t], config_.step_size);
      }
      stage_cost_ = cost_->get_stage_cost(x0_internal_, t0_internal_);
    }
    swap_policies();

    if (renderer_) renderer_->render(rollouts_);
    if (config_.logging) {
      // data_.rollouts = rollouts_;
      data_.step_count = step_count_;
      data_.stage_cost = stage_cost_;
      data_.rollouts_cost = rollouts_cost_;
      data_.weights = omega;
      data_.optimization_time = 0.0;
      data_.reset_time = t0_internal_;
      data_.optimal_rollout = opt_roll_;
      step_count_++;
    }
  }
}

void PathIntegral::time_it() {
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

void PathIntegral::set_observation(const observation_t& x, const double t) {
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

void PathIntegral::copy_observation() {
  std::shared_lock<std::shared_mutex> lock(state_mutex_);
  x0_internal_ = x0_;
  t0_internal_ = reset_time_;
}

void PathIntegral::initialize_rollouts() {
  std::shared_lock<std::shared_mutex> lock_state(state_mutex_);
  opt_roll_.clear();
  std::fill(opt_roll_.uu.begin(), opt_roll_.uu.end(),
            dynamics_->get_zero_input(x0_internal_));

  std::shared_lock<std::shared_mutex> lock(rollout_cache_mutex_);
  std::fill(opt_roll_cache_.xx.begin(), opt_roll_cache_.xx.end(), x0_internal_);
  std::fill(opt_roll_cache_.uu.begin(), opt_roll_cache_.uu.end(),
            dynamics_->get_zero_input(x0_internal_));
  for (size_t i = 0; i < steps_; i++) {
    opt_roll_cache_.tt[i] = t0_internal_ + config_.step_size * i;
  }
}

void PathIntegral::prepare_rollouts() {
  // find trim index
  size_t offset;
  {
    std::shared_lock<std::shared_mutex> lock(rollout_cache_mutex_);
    auto lower = std::lower_bound(opt_roll_cache_.tt.begin(),
                                  opt_roll_cache_.tt.end(), t0_internal_);
    if (lower == opt_roll_cache_.tt.end()) {
      std::stringstream warning;
      warning << "Resetting to time " << t0_internal_
              << ", greater than the last available time: "
              << opt_roll_cache_.tt.back();
      log_warning(warning.str());
    }
    offset = std::distance(opt_roll_cache_.tt.begin(), lower);
  }

  // sort rollouts for easier caching
  std::sort(rollouts_.begin(), rollouts_.end());

  // shift and trim so they restart from current time
  for (auto& roll : rollouts_) {
    shift_back(roll.uu, dynamics_->get_zero_input(roll.xx.back()), offset);
    roll.clear_cost();
    roll.clear_observation();
  }

  // shift and trim the previously optimized trajectory
  shift_back(opt_roll_.uu, dynamics_->get_zero_input(opt_roll_cache_.xx.back()),
             offset);
  shift_back(momentum_, momentum_.back(), offset);
}

void PathIntegral::set_reference_trajectory(mppi::reference_trajectory_t& ref) {
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

void PathIntegral::update_reference() {
  std::shared_lock<std::shared_mutex> lock(reference_mutex_);
  cost_->set_reference_trajectory(rr_tt_ref_);

  if (config_.threads > 1) {
    for (auto& cost : cost_v_) cost->set_reference_trajectory(rr_tt_ref_);
  }

  if (config_.use_tree_search) {
    tree_manager_.set_reference_trajectory(rr_tt_ref_);
  }
}

void PathIntegral::sample_noise(input_t& noise) { sampler_->get_sample(noise); }

void PathIntegral::sample_trajectories_batch(dynamics_ptr& dynamics,
                                             cost_ptr& cost,
                                             const size_t start_idx,
                                             const size_t end_idx) {
  observation_t x;
  for (size_t k = start_idx; k < end_idx; k++) {
    dynamics->reset(x0_internal_);
    x = x0_internal_;
    for (size_t t = 0; t < steps_; t++) {
      // cached rollout (recompute noise)
      if (k < cached_rollouts_) {
        rollouts_[k].nn[t] = rollouts_[k].uu[t] - opt_roll_.uu[t];
      }
      // noise free trajectory
      else if (k == cached_rollouts_) {
        rollouts_[k].nn[t].setZero();
        rollouts_[k].uu[t] = opt_roll_.uu[t];
      }
      // perturbed trajectory
      else {
        sample_noise(rollouts_[k].nn[t]);
        rollouts_[k].uu[t] = opt_roll_.uu[t] + rollouts_[k].nn[t];
      }

      // impose input constraints
      bound_input(rollouts_[k].uu[t]);

      // compute input-state stage cost
      double cost_temp = 0;
      cost_temp = std::pow(config_.discount_factor, t) *
                  cost->get_stage_cost(x, t0_internal_ + t * config_.step_size);

      if (std::isnan(cost_temp)) {
        throw std::runtime_error("Something went wrong ... dynamics diverged?");
      }

      // store data
      rollouts_[k].xx[t] = x;
      rollouts_[k].cc(t) = cost_temp;
      rollouts_[k].total_cost +=
          cost_temp -  // TODO(giuseppe) this should actually be a plus
          config_.lambda * opt_roll_.uu[t].transpose() * sampler_->sigma_inv() *
              rollouts_[k].nn[t] +
          config_.lambda * opt_roll_.uu[t].transpose() * sampler_->sigma_inv() *
              opt_roll_.uu[t];

      // integrate dynamics
      x = dynamics->step(rollouts_[k].uu[t], config_.step_size);
    }
    rollouts_cost_[k] = rollouts_[k].total_cost;
  }
}

void PathIntegral::sample_trajectories() {
  if (config_.threads == 1) {
    sample_trajectories_batch(dynamics_, cost_, 0, config_.rollouts);
  } else {
    for (size_t i = 0; i < config_.threads; i++) {
      futures_[i] = pool_->enqueue(
          std::bind(&PathIntegral::sample_trajectories_batch, this,
                    std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3, std::placeholders::_4),
          dynamics_v_[i], cost_v_[i],
          (size_t)i * config_.rollouts / config_.threads,
          (size_t)(i + 1) * config_.rollouts / config_.threads);
    }

    for (size_t i = 0; i < config_.threads; i++) futures_[i].get();
  }
}

void PathIntegral::sample_trajectories_via_tree() {
  tree_manager_.build_new_tree(x0_internal_, t0_internal_, opt_roll_);
}

void PathIntegral::overwrite_rollouts() {
  rollouts_ = tree_manager_.get_rollouts();
  rollouts_cost_ = tree_manager_.get_rollouts_cost();
}

void PathIntegral::compute_exponential_cost() {
  min_cost_ = rollouts_cost_.minCoeff();
  max_cost_ = rollouts_cost_.maxCoeff();

  if (config_.debug_print) print_cost_histogram();

  exponential_cost_ = Eigen::exp(-config_.h * (rollouts_cost_ - min_cost_) /
                                 (max_cost_ - min_cost_));
  if (config_.filtering) {
    for (size_t k = 0; k < config_.rollouts; k++) {
      if (rollouts_cost_(k) >
          (max_cost_ + (1 - config_.cost_ratio) * min_cost_))
        exponential_cost_(k) = 0.0;
    }
  }
}

void PathIntegral::optimize() {
  compute_exponential_cost();
  omega = exponential_cost_ / exponential_cost_.sum();

  // rollouts averaging
  for (size_t t = 0; t < steps_; t++) {
    momentum_[t] = config_.beta * momentum_[t];
    for (size_t k = 0; k < config_.rollouts; k++) {
      momentum_[t] += omega[k] * rollouts_[k].nn[t];
    }
  }

  for (size_t t = 0; t < steps_; t++) {
    opt_roll_.tt[t] = t0_internal_ + t * config_.step_size;
    opt_roll_.uu[t] += config_.alpha * momentum_[t];
  }

  // TODO(giuseppe) exploration covariance mean weighted noise covariance. Ok?
  if (config_.adaptive_sampling) {
    input_t delta = input_t::Zero(nu_);
    // TODO(giuseppe) remove the hardcoded baseline variance
    Eigen::MatrixXd new_covariance =
        Eigen::MatrixXd::Identity(nu_, nu_) * 0.001;
    for (size_t k = 0; k < config_.rollouts; k++) {
      for (size_t i = 0; i < steps_; i++) {
        delta = rollouts_[k].uu[i] - opt_roll_.uu[i];
        new_covariance += omega[k] * (delta * delta.transpose()) / steps_;
      }
    }
    // I could filter the covariance update using a first order
    // new_covariance = alpha * new_covariance + (1-alpha) old_covariance
    sampler_->set_covariance(new_covariance);
  }
}

void PathIntegral::filter_input() {
  if (config_.filter_type) {
    filter_.reset(t0_internal_);

    for (size_t i = 0; i < opt_roll_.uu.size(); i++) {
      filter_.add_measurement(opt_roll_.uu[i], opt_roll_.tt[i]);
    }

    for (size_t i = 0; i < opt_roll_.uu.size(); i++) {
      filter_.apply(opt_roll_.uu[i], opt_roll_.tt[i]);
    }
  }
}

void PathIntegral::get_input(const observation_t& x, input_t& u,
                             const double t) {
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
    // first
    if (idx == 0) {
      u = opt_roll_cache_.uu.front();
    }
    // last
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

void PathIntegral::get_diagonal_variance(Eigen::VectorXd& var) const {
  var = sampler_->sigma().diagonal();
}

void PathIntegral::get_input_state(const observation_t& x, observation_t& x_nom,
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

bool PathIntegral::get_optimal_rollout(observation_array_t& xx,
                                       input_array_t& uu) {
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

void PathIntegral::update_experts() { expert_.update_expert(1, opt_roll_.uu); };

void PathIntegral::swap_policies() {
  std::unique_lock<std::shared_mutex> lock(rollout_cache_mutex_);
  opt_roll_cache_ = opt_roll_;
}

PathIntegral::input_array_t PathIntegral::offline_control(
    const DynamicsBase::observation_t& x, const int subit, const double t) {
  set_observation(x, t);
  for (size_t i = 0; i < subit; i++) update_policy();
}

template <typename T>
void PathIntegral::shift_back(std::vector<T>& v_out, const T& fill,
                              const int offset) {
  std::rotate(v_out.begin(), v_out.begin() + offset, v_out.end());
  std::fill(v_out.end() - offset, v_out.end(), fill);
}

// explicit instantiation
template void PathIntegral::shift_back<Eigen::VectorXd>(
    std::vector<Eigen::VectorXd>& v_out, const Eigen::VectorXd& fill,
    const int offset);

void PathIntegral::print_cost_histogram() const {
  constexpr int nbins = 10;
  double delta = (max_cost_ - min_cost_) / nbins;

  std::cout << "Rollouts cost histogram" << std::endl;
  std::cout << "Max: " << max_cost_ << ", Min: " << min_cost_
            << ", delta: " << delta << std::endl;
  std::map<int, int> hist{};
  for (size_t k = 0; k < config_.rollouts; k++) {
    ++hist[std::round((rollouts_cost_(k) - min_cost_) / delta)];
  }
  for (auto p : hist) {
    std::cout << std::setw(2) << p.first << ' ' << std::string(p.second, '*')
              << '\n';
  }
  std::cout << std::endl << std::endl;
}

void PathIntegral::bound_input(input_t& u) {
  if (config_.bound_input)
    u = u.cwiseMax(config_.u_min).cwiseMin(config_.u_max);
}

}  // namespace mppi
