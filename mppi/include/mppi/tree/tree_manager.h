/*!
 * @file     tree_manager.h
 * @author   Etienne Walther
 * @date     08.12.2020
 * @version  1.0
 * @brief    description
 */

#pragma once

#include "mppi/utils/tree.h"

#include <iostream>
#include <future>
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <shared_mutex>

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

	/**
	 * @brief Tree Manager class
	 * @param dynamics: used to forward simulate the system and produce rollouts
	 * @param cost: to obtain running cost as function of the stage observation
	 * @param sampler: class used to draw random samples
	 * @param config: the solver configuration
	 * @param expert: used to make expert sampler available
	 */

  TreeManager(const dynamics_ptr& dynamics, cost_ptr cost, sampler_ptr sampler, config_t config, mppi::Expert *expert);
  ~TreeManager() = default;

public:

  /**
   * @brief Builds new tree structure for every optimization round
   * @param tree_dynamics_v: vector of dynamics (dim = n_rollouts) to improve multithreading
   * @param x0_internal: internal x0
   * @param t0_internal: internal t0
   * @param opt_roll: optimal rollout calculated from importance sampling
   */
  void build_new_tree(const std::vector<dynamics_ptr>& tree_dynamics_v, const observation_t& x0_internal, double t0_internal, const mppi::Rollout& opt_roll);

  /**
   * @brief Returns the rollouts sampled by the tree
   * @return vector of rollouts sampled by the tree
   */
	std::vector<mppi::Rollout> get_rollouts();

	/**
	 * @brief Returns the rollouts cost related to the rollouts sampled by the tree
	 * @return rollouts cost vector
	 */
  Eigen::ArrayXd get_rollouts_cost();

  /**
   * @brief Helper function to print the tree structure
   */
  void print_tree();

  /**
   * @brief Helper function to print the time it took from the start of building a new tree to the moment of the call of this function
   */
  void time_it();

 private:
  cost_ptr cost_;
  dynamics_ptr dynamics_;
  config_t config_;
  sampler_ptr sampler_;
  expert_ptr expert_;

  tree<Node> sampling_tree_; // The tree object
  std::unique_ptr<ThreadPool> pool_; // The thread pool

  std::vector<dynamics_ptr> tree_dynamics_v_; //
  std::vector<dynamics_ptr> tree_dynamics_v_shared_;

  /**
   * @brief Initialization of a variables related to a new tree
   */
  void init_tree();

  /**
   * @brief Called after initalization to add and evaluate the depth levels
   */
  void grow_tree();

  /**
   * @brief Initialization of threading
   */
  void init_threading();

  /**
   * @brief Transforms the tree into the rollouts structure used by mppi
   */
  void transform_to_rollouts();

  /**
   * @brief Adds a new depth level to the tree.
   * @param horizon_step: starts at 1 to properly handle time in rollouts
   */
  void add_depth_level(size_t horizon_step);

  /**
   * @brief Evaluates the last depth level added.
   */
  void eval_depth_level();

  /**
   * @brief Creates the mapping between the rollout index and the expert type
   * @param mapping_type_input: specifies what type of mapping should be used. Currently only mapping_type_input=0 is supported.
   */
  void set_rollout_expert_mapping(size_t mapping_type_input);

  /**
   * @brief Adds a node to the tree by sampling a control input from an expert, stepping the dynamics. Internally manages from which branch it should be forked. Thread-Safe.
   * @param horizon_step: specifies on what horizonstep the process is (saved in Node)
   * @param leaf_pos: specifies the rollout index
   * @param node_dynamics: one dynamics instance from the dynamics vector used for multithreading is assigned to a node
   * @return Returns an iterator which points to the position in the tree
   */
  tree<Node>::iterator add_node(size_t horizon_step, size_t leaf_pos, dynamics_ptr node_dynamics);

  /**
   * @brief Helper function to sample from the uniform distribution in the range [v_min, v_max]
   * @param v_min: lower bound
   * @param v_max: upper bound
   * @return Returns a sample from the uniform distribution in the range [v_min, v_max]
   */
  static int random_uniform_int(int v_min, int v_max);

private:
  double t0_internal_;
  observation_t x0_internal_;
  mppi::Rollout opt_roll_;

  double t_;
  std::size_t tree_width_;
  std::size_t tree_target_depth_;

  std::map<size_t, size_t> rollout_expert_map_; // Mapping of rollout indexes to experts

  std::vector<size_t> extendable_leaf_pos_; // Set of extendable leafs

  std::chrono::high_resolution_clock::time_point start_time_;

  std::vector<tree<Node>::iterator> leaf_handles_; // Vector of leaf handles
  std::vector<std::future<tree<Node>::iterator>> futures_; // vector of futures, such that every thread can return the leaf handle of it's newly created leaf

  std::vector<mppi::Rollout> rollouts_;
  Eigen::ArrayXd rollouts_cost_;

  std::shared_mutex tree_mutex_; // Tree mutex to only allow access to the tree by one thread at the time

};


