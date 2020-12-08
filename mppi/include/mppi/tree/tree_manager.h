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
#include "mppi/utils/tree.h"
#include "node.h"
#include "mppi/utils/thread_pool.h"
#include <Eigen/Dense>


class TreeManager {
 public:
  using cost_ptr = mppi::CostBase::cost_ptr;
  TreeManager(cost_ptr cost);

  ~TreeManager() = default;

  void print_tree();

  void time_it();


 private:
  cost_ptr cost_;

  const size_t NUM_THREADS_ = std::thread::hardware_concurrency();

  tree<Node> sampling_tree;
  std::unique_ptr<ThreadPool> pool_;

  void init_tree();

  void build_tree();

  void add_depth_level();

  tree<Node>::iterator add_node(tree<Node>::iterator parent_handle);

  std::size_t tree_width_ = 0;
  std::size_t tree_depth_ = 0;

  std::chrono::high_resolution_clock::time_point start_time_;

  std::vector<tree<Node>::iterator> leaf_handles;
  std::vector<std::future<tree<Node>::iterator>> futures_;

  void init_threading();

  void transform_to_rollouts();
};


