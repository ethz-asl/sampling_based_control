/*!
 * @file     tree_manager.cpp
 * @author   Etienne Walther
 * @date     08.12.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi/tree/tree_manager.h"

TreeManager::TreeManager(cost_ptr cost) : sampling_tree(){
  cost_ = cost;
  init_tree();
  init_threading();
  build_tree();
  transform_to_rollouts();
}

void TreeManager::init_tree(){
  start_time_ = std::chrono::high_resolution_clock::now();
  tree_width_ = 2; //config: rollouts
  tree_depth_ = 3; //config: horizon

  futures_.resize(tree_width_);
  std::cout << sizeof(futures_);

  auto root = sampling_tree.insert(sampling_tree.begin(), Node(0,0,cost_));

  leaf_handles.resize(tree_width_);

  for (int leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos) {
    leaf_handles[leaf_pos] = sampling_tree.append_child(root, Node(0,0,cost_));
  }
}

void TreeManager::print_tree() {
  tree<Node>::iterator start_node = tree<Node>::begin(sampling_tree.begin());
  tree<Node>::iterator end_node = tree<Node>::end(sampling_tree.end());

  do {
    int node_depth = tree<Node>::depth(start_node);

    for (int i = 0; i < node_depth; ++i) {
      std::cout << " ";
    }

    std::cout << start_node->public_name_ << std::endl;

    start_node++;
  } while (start_node != end_node);
}

void TreeManager::add_depth_level() {
  for (int leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos) {
    auto active_leaf = leaf_handles[leaf_pos];
    futures_[leaf_pos] = pool_->enqueue(bind(&TreeManager::add_node,this, std::placeholders::_1), active_leaf);
  }

  //auto future_elem = futures_.begin();
  for (size_t leaf_pos = 0; leaf_pos < tree_width_; leaf_pos++) leaf_handles[leaf_pos] = futures_[leaf_pos].get();

}

void TreeManager::build_tree() {

  auto max_depth_ = sampling_tree.depth(sampling_tree.end());

  for (int i = 0; i < tree_depth_; ++i) {
    add_depth_level();
  }
}

tree<Node>::iterator TreeManager::add_node(tree<Node>::iterator parent_handle) {

  unsigned int microsecond = 1;
  usleep(20* microsecond);
  return sampling_tree.append_child(parent_handle, Node(0,0,cost_));
}

void TreeManager::init_threading() {
  pool_ = std::make_unique<ThreadPool>(NUM_THREADS_);
}

void TreeManager::time_it(){
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start_time_).count();
  std::cout << "Time since tree initialization: " << duration << "[μs], " << duration/1000000.0 << "[s], " << 1/(duration/1000000.0) << "[Hz]" << std::endl;
}

void TreeManager::transform_to_rollouts(){
  for (auto leaf_handle : leaf_handles){
    auto path_to_leaf = sampling_tree.path_from_iterator(leaf_handle, sampling_tree.begin());

    for (size_t i = 0; i < path_to_leaf.size(); ++i ){
      std::vector<int> path_to_leaf_cut(path_to_leaf.begin(), path_to_leaf.begin()+i);
      auto value = sampling_tree.iterator_from_path(path_to_leaf_cut, sampling_tree.begin())->public_name_;
    }
  }
}

