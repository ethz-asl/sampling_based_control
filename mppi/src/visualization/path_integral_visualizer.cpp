/*!
 * @file     path_integral_visualizer.cpp
 * @author   Giuseppe Rizzi
 * @date     22.07.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi/visualization/path_integral_visualizer.h"
#include <chrono>
#include <thread>

namespace mppi {

state_input_pair PathIntegralVisualizer::run(const observation_t &x,
                                             const double t, int step_count) {
  if (!initialized) init_viewer();

  static state_input_pair state_input;

  static bool visualize;
  if ((size_t)(fmod(step_count, step_stride)) == 0)
    visualize = true;
  else
    visualize = false;

  state_input.u = step(x, t, visualize, step_count);
  dynamics_->reset(x);
  state_input.x = dynamics_->step(state_input.u, config_.step_size);
  return state_input;
}

PathIntegralVisualizer::input_t PathIntegralVisualizer::step(
    const observation_t &x, const double t, bool show, size_t step_count) {
  set_observation(x, t);
  update_policy();
  rollout_counter_ = 0;

  if (show) {
    std::cout << std::string(100, '=') << std::endl;
    std::cout << "Step " << step_count << std::endl;
    std::cout << std::string(100, '=') << std::endl;
    handle_user_input();
    show_optimal_path(x);
  }
  input_t u;
  get_input(x, u, t);
  return u;
}

void PathIntegralVisualizer::init_viewer() {
  std::cout << std::string(100, '-') << std::endl;
  std::cout << "Welcome to Path Integral Viewer" << std::endl;
  std::cout << std::string(100, '-') << std::endl;
  std::cout << "Enter the step_stride [>=0] factor per dynamics simulation."
            << std::endl;
  std::cin >> step_stride;
  step_stride = step_stride < 1 ? 1 : (size_t)std::ceil(step_stride);
  std::cout << "Enter the speed up [>=1] factor per rollout visualization."
            << std::endl;
  std::cin >> speed_up;
  speed_up = speed_up < 1 ? 1 : speed_up;
  std::cout << "Enter the pause time [sec] between rollout visualization."
            << std::endl;
  std::cin >> pause_between_rollout;
  std::cout << "Enter the rollout stride per rollout visualization."
            << std::endl;
  std::cin >> rollout_stride;
  std::cout
      << "Default visualization (slideshow and no optimal path repeat) [y/n]"
      << std::endl;
  char answer;
  std::cin >> answer;
  if (answer == 'y') default_visualization = true;
  rollout_stride = rollout_stride == 0 ? 1 : rollout_stride;
  initialized = true;
}

void PathIntegralVisualizer::handle_user_input() {
  std::cout << std::string(100, '-') << std::endl;
  static char c;
  if (default_visualization) {
    c = 's';
  } else {
    std::cout << "Enter: [n]=next rollout, [s]=rollout slide show, [m]=all "
                 "trajectories, [e]=exit, [i]=show rollout info"
              << std::endl;
    std::cin >> c;
  }
  switch (c) {
    case 'n': {
      if (rollout_counter_ > rollouts_.size()) {
        std::cout << "No more rollouts, optimizing input for next iteration."
                  << std::endl;
        return;
      }
      visualize_single_trajectory(rollouts_[rollout_counter_].xx,
                                  config_.step_size / speed_up);
      rollout_counter_++;
      handle_user_input();
      break;
    }
    case 'i': {
      std::cout << "Rollout info." << std::endl;
      print_rollout_info();
      print_weight_info();
      handle_user_input();
      break;
    }
    case 's': {
      for (size_t i = rollout_counter_; i < rollouts_.size();
           i += rollout_stride) {
        std::cout << "Rollout [" << i << "], cost=" << rollouts_cost_(i)
                  << ", weight=" << omega(i) << std::endl;
        visualize_single_trajectory(rollouts_[i].xx,
                                    config_.step_size / speed_up);
        if (pause_between_rollout > 0)
          std::this_thread::sleep_for(
              std::chrono::milliseconds((int)(pause_between_rollout * 1000)));
      }
      return;
    }
    case 'm': {
      std::cout << "Visualizing all the planned trajectories " << std::endl;
      visualize_all_trajectories();
      handle_user_input();
      break;
    }
    case 'e': {
      std::cout << "Exiting rollout viewer" << std::endl;
      return;
    }
    default: {
      std::cout << "Entered invalid prompt";
      handle_user_input();
    }
  }
}

void PathIntegralVisualizer::show_optimal_path(const observation_t &x0) {
  observation_t x;
  observation_array_t path;
  path.resize(steps_);
  dynamics_->reset(x0);
  std::cout << std::string(100, '*') << std::endl;
  std::cout << "Rolling out optimized input..." << std::endl;
  for (size_t i = 0; i < steps_; i++) {
    path[i] = dynamics_->step(opt_roll_cache_.uu[i], config_.step_size);
  }
  char c = 'y';
  while (c == 'y') {
    visualize_optimal_trajectory(path);
    if (default_visualization) return;
    std::cout << "Repeat trajectory visualization? [y/n] " << std::endl;
    std::cin >> c;
  }
}

void PathIntegralVisualizer::print_rollout_info() {
  static double min_cost;
  static double max_cost;
  static double delta;
  static size_t bins = 10;
  min_cost = get_rollout_min_cost();
  max_cost = get_rollout_max_cost();
  delta = (max_cost - min_cost) / bins;

  std::cout << "Rollouts cost: " << rollouts_cost_.transpose() << std::endl;
  std::cout << "Rollouts cost histogram" << std::endl;
  std::cout << "Max: " << max_cost << ", Min: " << min_cost
            << ", delta: " << delta << std::endl;
  std::map<int, int> hist{};
  for (size_t k = 0; k < config_.rollouts; k++) {
    ++hist[std::round((rollouts_cost_(k) - min_cost) / delta)];
  }
  for (auto p : hist) {
    std::cout << std::setw(2) << p.first << ' ' << std::string(p.second, '*')
              << '\n';
  }
}

void PathIntegralVisualizer::print_weight_info() {
  static double min_weight;
  static double max_weight;
  static double delta;
  static size_t bins = 100;
  min_weight = get_weight_min_cost();
  max_weight = get_weight_max_cost();
  delta = 1.0 / bins;

  std::cout << "Rollouts weight plot" << std::endl;
  std::cout << std::setprecision(5) << "Max: " << max_weight
            << ", Min: " << min_weight << std::endl;
  std::map<int, int> hist{};
  for (int i = 0; i < bins; i++) hist[i] = 0;
  for (int k = 0; k < config_.rollouts; k++) {
    ++hist[std::round(omega(k) / delta)];
  }
  for (auto p : hist) {
    if (p.second)
      std::cout << std::setw(4) << std::setprecision(3)
                << (p.first / bins + 1) * delta << ' '
                << std::string(p.second, '*') << '\n';
  }
}

void PathIntegralVisualizer::set_default_view_if_true(bool flag) {
  std::cout << "Setting the default view? " << flag << std::endl;
  default_visualization = flag;
}

}  // namespace mppi
