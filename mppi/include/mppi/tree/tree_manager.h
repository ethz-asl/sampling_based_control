/*!
 * @file     tree_manager.h
 * @author   Etienne Walther
 * @date     08.12.2020
 * @version  1.0
 * @brief    description
 */

#pragma once

#include "mppi/utils/tree.h"

#include <Eigen/Dense>
#include <chrono>
#include <future>
#include <iostream>
#include <shared_mutex>
#include <thread>

#include "mppi/controller/rollout.h"
#include "mppi/experts/expert.h"
#include "mppi/sampler/gaussian_sampler.h"
#include "mppi/utils/data_logger.h"
#include "mppi/utils/thread_pool.h"
#include "mppi/utils/tree.h"
#include "node.h"

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
   * @param dynamics: used to simulate the system and produce rollouts
   * @param cost: to obtain running cost as function of the stage observation
   * @param sampler: class used to draw random samples
   * @param config: the solver configuration
   * @param expert: used to make expert samples available to the tree manager
   */

  TreeManager(const dynamics_ptr& dynamics, cost_ptr cost, sampler_ptr sampler, config_t config,
              mppi::Expert* expert);
  ~TreeManager() = default;

 public:
  /**
   * @brief Builds new tree structure. Called at the beginning of every policy update.
   * @param x0_internal: internal x0
   * @param t0_internal: internal t0
   * @param opt_roll: optimal rollout calculated from importance sampling
   */
  void build_new_tree(const observation_t& x0_internal, double t0_internal,
                      const mppi::Rollout& opt_roll);

  /**
   * @brief Returns the rollouts sampled by the tree
   * @return vector of rollouts sampled by the tree
   */
  std::vector<mppi::Rollout> get_rollouts();

  /**
   * @brief Returns the rollouts cost related to the rollouts sampled by the tree
   * @return vector of costs for rollouts
   */
  Eigen::ArrayXd get_rollouts_cost();

  /**
   * @brief Helper function to print the tree structure
   */
  void print_tree();

 private:
  cost_ptr cost_;
  dynamics_ptr dynamics_;
  config_t config_;
  sampler_ptr sampler_;
  expert_ptr expert_;

  tree<Node> sampling_tree_;                   // The tree object
  std::unique_ptr<ThreadPool> pool_;           // The thread pool for sampling the control inputs
  std::unique_ptr<ThreadPool> pool_add_leaf_;  // The thread pool for appending leafs to the tree

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
   * @param mapping_type_input: specifies what type of mapping should be used. Currently only
   * mapping_type_input=0 is supported.
   */
  void gen_rollout_expert_mapping(size_t mapping_type_input);

  /**
   * @brief Samples a control input from an expert and steps the dynamics for one rollout.
   * Internally manages from which branch it should be forked. Thread-Safe. Calls
   * append_child_to_tree_via_pool to add new leaf to the tree
   * @param horizon_step: specifies on what horizonstep the process is (saved in Node)
   * @param leaf_pos: specifies the rollout index
   * @param node_dynamics: one dynamics instance from the dynamics vector used for multithreading is
   * assigned to a node
   * @return Returns an iterator which points to the position in the tree
   */
  bool add_node(size_t horizon_step, size_t leaf_pos, const dynamics_ptr& node_dynamics,
                const expert_ptr& node_expert);

  /**
   * @brief Adds new leaf to the tree. This is non-thread-safe and therefore the tree needs to be
   * locked by a mutex
   * @return leaf handle of added leaf
   */
  tree<Node>::iterator append_child_to_tree_via_pool(
      tree<Node>::iterator extending_leaf, size_t horizon_step, const Eigen::VectorXd& u_applied,
      const Eigen::VectorXd& x, const Eigen::MatrixXd& sigma_inv, size_t expert_type_applied);

  /**
   * @brief Helper function to sample from the uniform distribution in the range [v_min, v_max]
   * @param v_min: lower bound
   * @param v_max: upper bound
   * @return Returns a sample from the uniform distribution in the range [v_min, v_max]
   */
  static int random_uniform_int(int v_min, int v_max);

  /**
   * @brief Trims the input to the allowed upper and lower bound
   * @param u: input
   * @return trimmed input u
   */
  Eigen::VectorXd bound_input(input_t& u);

 private:
  double t0_internal_;         // copy of internal time from PathIntegral
  observation_t x0_internal_;  // copy of internal state from PathIntegral
  mppi::Rollout opt_roll_;     // copy of optimal rollout from PathIntegral

  double t_;                       // internal time at for horizon step
  std::size_t tree_width_;         // target tree witdth
  std::size_t tree_target_depth_;  // target tree depth

  std::map<size_t, size_t> rollout_expert_map_;  // Mapping of rollout indexes to experts

  std::vector<size_t> extendable_leaf_pos_;  // Set of extendable leafs

  std::vector<tree<Node>::iterator> leaf_handles_;  // Vector of leaf handles
  std::vector<std::future<bool>>
      futures_;  // futures vector of booleans. Every thread returns true when control input sampled
                 // and applied to the dynamics
  std::vector<std::future<tree<Node>::iterator>>
      futures_add_leaf_to_tree_;  // futures vector of leaf handles. Every thread returns the leaf
                                  // handle of the appended leaf to the tree

  std::vector<mppi::Rollout> rollouts_;  // internal tree rollouts
  Eigen::ArrayXd rollouts_cost_;         // internal tree rollout costs

  std::shared_mutex
      tree_mutex_;  // Tree mutex to only allow access to the tree by one thread at the time

  std::vector<dynamics_ptr>
      tree_dynamics_v_;  // tree_dynamics_v_[leaf_position] assigned to every node at beginning
  std::vector<dynamics_ptr>
      tree_dynamics_v_shared_;  // all nodes save the resulting dynamics to this vector.

  std::vector<expert_ptr>
      experts_v_;  // vector of experts such that every thread can sample independently
};
