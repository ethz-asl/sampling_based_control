/*!
 * @file     PathIntegralVisualizer.h
 * @author   Giuseppe Rizzi
 * @date     22.07.2020
 * @version  1.0
 * @brief    description
 */
#pragma once
#include <vector>
#include "mppi/controller/mppi.h"

namespace mppi {

struct state_input_pair {
  mppi::PathIntegral::observation_t x;
  mppi::PathIntegral::input_t u;
};

class PathIntegralVisualizer : public PathIntegral {
 public:
  using observation_array_t = std::vector<observation_t>;
  PathIntegralVisualizer() = delete;
  PathIntegralVisualizer(DynamicsBase::dynamics_ptr dynamics,
                         CostBase::cost_ptr cost,
                         GaussianSampler::sampler_ptr sampler,
                         const SolverConfig& config)
      : PathIntegral(dynamics, cost, config, sampler) {}
  ~PathIntegralVisualizer() = default;

 public:
  /**
   * @brief Function implementing the trajectory visualization
   * @param traj: single rollout state trajectory
   * @param dt: how ling to sleep in between waypoints
   */
  virtual void visualize_single_trajectory(const observation_array_t& traj,
                                           double dt = 0) = 0;

  /**
   * @brief Same as visualization for single trajectory, specialized on the
   * optimized one
   * @param traj: optimal trajectory rollout
   */
  virtual void visualize_optimal_trajectory(
      const observation_array_t& traj) = 0;

  /**
   * @brief Self explanatory
   */
  virtual void visualize_all_trajectories() {
    std::cout << "Not implemented." << std::endl;
  }

  /**
   * @brief Print to console the rollouts info such as histogram, cost, min and
   * max cost
   */
  void print_rollout_info();

  /**
   * @brief Print to console the rollouts weights histogram
   */
  void print_weight_info();

  /**
   * @brief Set the default visualization
   * @param flag
   */
  void set_default_view_if_true(bool flag);

  /**
   * @brief Run one simulation step with additional visualization
   * @param x: the current observation
   * @param t: current time
   * @param step_count: simulation/control step
   * @return
   */
  state_input_pair run(const observation_t& x, const double t, int step_count);

 private:
  /**
   * @brief Step the visualization for the current observation and time
   * @param x: current system observation
   * @param t: current time
   * @param show: show trajectories
   * @param step_count: current step in the simulation (theoretically
   * (int)t/step_size)
   */
  input_t step(const observation_t& x, const double t, bool show,
               size_t step_count);

  /**
   * @brief Ask the user for some viewer parameters
   */
  void init_viewer();

  /**
   * @brief Handle user input from console for interactive visualization
   */
  void handle_user_input();

  /**
   * @brief Show the rollout corresponding to the optimized path
   * @param x: the initial state
   */
  void show_optimal_path(const observation_t& x);

 protected:
  bool initialized = false;
  bool default_visualization = false;
  size_t rollout_counter_ = 0;
  int rollout_stride = 0;
  size_t step_stride = 0;
  double speed_up = 1.0;
  double pause_between_rollout = 0.0;
};

}  // namespace mppi
