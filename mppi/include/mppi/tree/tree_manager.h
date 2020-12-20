/*!
 * @file     tree_manager.h
 * @author   Etienne Walther
 * @date     08.12.2020
 * @version  1.0
 * @brief    description
 */

#pragma once

#include "mppi/utils/tree.h"

#include <cstdio>
#include <iostream>
#include <future>
#include <thread>
#include <unistd.h>
#include <chrono>
#include <Eigen/Dense>

#include "mppi/utils/tree.h"
#include "node.h"
#include "mppi/utils/thread_pool.h"
#include "mppi/sampler/gaussian_sampler.h"
#include "mppi/experts/expert.h"
#include "mppi/controller/rollout.h"



class TreeManager {
 public:
  using cost_ptr = mppi::CostBase::cost_ptr;
  using dynamics_ptr = mppi::DynamicsBase::dynamics_ptr;
  using config_t = mppi::SolverConfig;
  using observation_t = mppi::DynamicsBase::observation_t;
  using input_t = mppi::DynamicsBase::input_t;
  using sampler_ptr = mppi::GaussianSampler::sampler_ptr;
  using expert_ptr = mppi::Expert::expert_ptr;

  TreeManager(cost_ptr cost, dynamics_ptr dynamics, config_t config, sampler_ptr sampler, mppi::Expert *expert);

  ~TreeManager() = default;

  void print_tree();

  void time_it();

  void build_new_tree(std::vector<dynamics_ptr> tree_dynamics_v, const observation_t x0_internal, double t0_internal, mppi::Rollout opt_roll);

	std::vector<mppi::Rollout> get_rollouts();


 private:
  cost_ptr cost_;
  dynamics_ptr dynamics_;
  config_t config_;
  sampler_ptr sampler_;
  expert_ptr expert_;

  tree<Node> sampling_tree_;
  std::unique_ptr<ThreadPool> pool_;

  std::vector<dynamics_ptr> tree_dynamics_v;
  std::vector<dynamics_ptr> tree_dynamics_next_v;

  void init_tree();
  void grow_tree();

  void add_depth_level(size_t horizon_step);

  void eval_depth_level();

  tree<Node>::iterator add_node(size_t horizon_step, size_t leaf_pos);

  std::size_t tree_width_;
  std::size_t tree_target_depth_;

  std::chrono::high_resolution_clock::time_point start_time_;

  std::vector<tree<Node>::iterator> leaf_handles_;
  std::vector<std::future<tree<Node>::iterator>> futures_;

  void init_threading(size_t num_threads);

  void transform_to_rollouts();

  void reset();

  double t_;

  int random_uniform_int(int v_min, int v_max);

  double random_uniform_double(double v_min, double v_max);

  std::vector<size_t> extendable_leaf_pos_;

  std::map<size_t, size_t> rollout_expert_map_;

  void set_rollout_expert_mapping(size_t mapping_type_input);

  std::vector<mppi::Rollout> rollouts_;

  // start time and location of new tree
  double t0_internal_;
  observation_t x0_internal_;
  mppi::Rollout opt_roll_;


};


