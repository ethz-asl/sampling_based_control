/*!
 * @file     mppi.h
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */
#pragma once
#include <algorithm>
#include <boost/filesystem.hpp>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <shared_mutex>
#include <vector>

#include <Eigen/Core>

#include "mppi/core/config.h"
#include "mppi/core/cost.h"
#include "mppi/core/data.h"
#include "mppi/core/dynamics.h"
#include "mppi/core/filter.h"
#include "mppi/core/policy.h"
#include "mppi/core/rollout.h"

#include "mppi/utils/gaussian_sampler.h"
#include "mppi/utils/savgol_filter.h"
#include "mppi/utils/thread_pool.h"
#include "mppi/utils/timer.h"

namespace mppi {

class Solver {
 public:
  /**
   * @brief Path Integral Control class
   * @param dynamics: used to forward simulate the system and produce rollouts
   * @param cost: to obtain running cost as function of the stage observation
   * @param policy: implementation of stochastic policy
   * @param config: the solver configuration
   * @param verbose: (bool) flag to turn on/off verbosity
   */
  Solver(dynamics_ptr dynamics, cost_ptr cost, policy_ptr policy,
         const config_t& config);
  Solver() = default;
  ~Solver() = default;

 private:
  /**
   * @brief Init the filter if required
   */
  void init_filter();

  /**
   * @brief Init the data structures used during the computation
   */
  void init_data();

  /**
   * @brief Initializes the variable for multithreading is required (threads >
   * 1)
   */
  void init_threading();

  /**
   * @brief Calculates and displays the frequency of the control loop
   */
  void time_it();

 public:
  void set_filter(filter_ptr& filter) { filter_ = filter; }

  /**
   * @brief Filter the input according to chosen filter
   */
  void filter_input();

  /**
   * @brief Transforms the cost to go into the corresponding sample weights
   * @return
   */
  void compute_weights();

  /**
   * @brief First multiple rollouts are forward simulated and averaged to
   * compute a newly refined control input
   * @param x current observation
   */
  void update_policy();

  /**
   * @brief Sample a noise vector to add to input for exploration
   * @param noise the noise vector (same dimension as the input space)
   */
  void sample_noise(input_t& noise);

  /**
   * @brief Sample a batch of trajectories
   * @param dynamics the dynamics function to use
   * @param cost the cost function to use
   * @param start_idx first index in the rollouts vector
   * @param end_idx last index in the rollouts vector
   */
  void sample_trajectories_batch(dynamics_ptr& dynamics, cost_ptr& cost,
                                 const size_t start_idx, const size_t end_idx);

  /**
   * @brief Sample multiple trajectories starting from current observation and
   * applying noise input with mean the previous shifted optimal input. A ratio
   * of best previous rollouts is reused to warm start new sample generation and
   * one rollout is a noise free one.
   */
  void sample_trajectories();

  /**
   * @brief Use the data collected from rollouts and return the optimal input
   */
  void optimize();

  /**
   * @brief Prepare rollouts before the new optimization starts.
   * @details The previous optimal rollout as well a portion of cached rollouts
   * (according to the caching_factor) get trimmed up to the new reset time. The
   * remaining are cleared to be overwritten in the next control loop. Cached
   * rollouts has different additive noise wrt new updated optimal input, thus
   * this needs to be recomputed
   */
  void prepare_rollouts();

  /**
   * @brief Shift the control vector of one element back for receding horizon
   * control
   * @param v_out: vector to shift back
   * @param fill: object used to fill the remaining part of the vector
   * @param offset: how much to translate the vector
   */
  template <typename T>
  void shift_back(std::vector<T>& v_out, const T& fill = T(),
                  const int offset = 1);

  /**
   * @brief Print to screen the cost histogram split in a fixes number of bins
   */
  void print_cost_histogram() const;

  /**
   * @brief Initializes rollouts for the first time
   */
  void initialize_rollouts();

  /**
   * @brief Fill the optimized policy cache with the latest policy and set flags
   */
  void swap_policies();

  /**
   * @brief Estimate optimization delay
   */
  void update_delay();

  /**
   * @brief Return the latest optimal input for the current time and observation
   * @param x[in, unused]: current observation
   * @param u[out]: optimal input
   * @param t[int]: current time
   */
  void get_input(const observation_t& x, input_t& u, const double t);

  /**
   * @brief Get the nominal input/state at time t given current observation
   * @param x : current observation
   * @param x_nom : nominal state
   * @param u_nom : nominal input
   * @param t : current time
   */
  void get_input_state(const observation_t& x, observation_t& x_nom,
                       observation_t& u_nom, const double t);

  /**
   * @brief Returns the optimal rollout from the latest updated time when
   * calling set_observation
   * @param x the state trajectory
   * @param u the input trajectory
   * @return false if time is later then the latest available in the nominal
   * rollout
   */
  bool get_optimal_rollout(observation_array_t& x, input_array_t& u);

  /**
   * @brief Another signature to get the optimal rollout
   */
  void get_optimal_rollout(Rollout& r);
  
  /**
   * @brief Get only the diagonal of the sampler's covariance matrix
   * @param var[in/out]: the diagonal variance
   */
  void get_diagonal_variance(Eigen::VectorXd& var) const;

  /**
   * @brief Reset the initial observation (state) and time for next optimization
   * @param x: current observation
   * @param t: current time
   */
  void set_observation(const observation_t& x, const double t);

  /**
   * @brief Copy latest observation and time to internal variables used for the
   * next optimization
   * @details Copying latest x and t to internal variables is required to
   * perform the next optimization starting from the same initial state and time
   * while the public one can be potentially changed by a different thread.
   */
  void copy_observation();
  /**
   * @brief Set the reference of the cost function to the latest received for
   * the new optimization
   */
  void update_reference();

  // Generic getters
  inline const std::vector<double>& get_weights() const { return weights_; }
  inline double get_stage_cost() { return stage_cost_; }
  inline double get_rollout_cost() { return get_rollout_min_cost(); }
  inline double get_rollout_min_cost() { return 0.0; }
  inline double get_rollout_max_cost() { return 0.0; }
  inline const Rollout& get_optimal_rollout() { return opt_roll_; }
  inline const Rollout& get_optimal_rollout_cache() { return opt_roll_cache_; }
  inline observation_t get_current_observation() { return x0_internal_; }
  inline data_t get_data() { return data_; }

 public:
  /**
   * @brief Set the reference trajectory to be used in the next optimization
   * loop
   * @param ref: reference trajectory
   */
  void set_reference_trajectory(reference_trajectory_t& ref);

  // TODO clean this up: everything public...
 public:
  bool verbose_;
  std::vector<double> weights_;
  Eigen::ArrayXd exponential_cost_;

  dynamics_ptr dynamics_;
  cost_ptr cost_;
  policy_ptr policy_;
  config_t config_;
  sampler_ptr sampler_;

  size_t cached_rollouts_;

  std::vector<Rollout> rollouts_;
  int steps_;

  bool first_step_ = true;

 protected:
  // time from which the current optimization has started
  double reset_time_;
  // first state for simulation
  observation_t x0_;
  // utility variable, store the currently simulated step state
  observation_t x_;
  // internal t0 for next optimization
  double t0_internal_;
  // internal x0 for next optimization
  observation_t x0_internal_;
  // state dimension
  size_t nx_;
  // input dimension
  size_t nu_;
  // optimized rollout
  Rollout opt_roll_;
  // previously optimized rollout
  Rollout opt_roll_cache_;
  // min cost among all rollouts
  double min_cost_;
  // max cost among all rollouts
  double max_cost_;
  // flag to check that observation has ever been set
  std::atomic_bool observation_set_;
  // protects access to the solution
  std::shared_mutex rollout_cache_mutex_;
  // protects access to the state
  std::shared_mutex state_mutex_;
  // flag to check that reference has ever been set
  std::atomic_bool reference_set_;
  // protects access to the reference trajectory
  std::shared_mutex reference_mutex_;
  // reference used during optimization
  reference_trajectory_t rr_tt_ref_;
  // thread pool
  std::unique_ptr<ThreadPool> pool_;
  // futures results from the thread pool
  std::vector<std::future<void>> futures_;
  // vector of dynamics functions used per each thread
  std::vector<dynamics_ptr> dynamics_v_;
  // vector of cost functions used per each thread
  std::vector<cost_ptr> cost_v_;
  // summary
  data_t data_;
  // stage_cost used for logging
  double stage_cost_ = 0;
  // time to measure the frequency of the control loop
  std::chrono::high_resolution_clock::time_point start_time_;
  // total amount of solver optimization steps
  size_t step_count_;
  // momentum term (dim = input_dim x steps)
  std::vector<input_t> momentum_;
  // timer
  Timer timer_;
  // track indeces of rollouts which did not diverge
  std::vector<int> rollouts_valid_index_;
  // rollouts costs
  std::vector<double> rollouts_cost_;

  // delay estimation
  int delay_steps_;
  int new_delay_steps_;
  const double delay_filter_alpha_ = 0.9;

  // filter input
  Eigen::MatrixXd nominal_;  // the nominal u-traj after filtering to reset the
                             // policy warm start
  filter_ptr filter_;

  // rate measurements
  double rate_;
};

}  // namespace mppi
