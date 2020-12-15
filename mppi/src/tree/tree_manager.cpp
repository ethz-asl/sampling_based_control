/*!
 * @file     tree_manager.cpp
 * @author   Etienne Walther
 * @date     08.12.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi/tree/tree_manager.h"

TreeManager::TreeManager(cost_ptr cost, dynamics_ptr dynamics, config_t config, sampler_ptr sampler, expert_ptr expert) : sampling_tree_(){
  cost_ = cost;
  dynamics_ = dynamics;
  config_ = config;
  sampler_ = sampler;
  expert_ = expert;

  set_rollout_expert_mapping(0);

  init_threading(config_.threads);
}

void TreeManager::init_tree(observation_t x0_internal){
  start_time_ = std::chrono::high_resolution_clock::now();
  tree_width_ = config_.rollouts;
  tree_target_depth_ = config_.horizon;

  futures_.resize(tree_width_);

  tree<Node>::iterator root = sampling_tree_.insert(sampling_tree_.begin(), Node(nullptr, 0, config_, cost_, x0_internal, dynamics_->get_zero_input(x0_internal)));

  leaf_handles_.resize(tree_width_);
	extendable_leaf_pos_.resize(tree_width_);

  for (size_t leaf_pos=0; leaf_pos < tree_width_; ++leaf_pos){
		extendable_leaf_pos_[leaf_pos] = leaf_pos;
  }

  for (int leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos) {
    leaf_handles_[leaf_pos] = sampling_tree_.append_child(root, Node(root->parent_node_,0,config_, cost_, x0_internal, dynamics_->get_zero_input(x0_internal)));
  }
}

void TreeManager::print_tree() {
  tree<Node>::iterator start_node = tree<Node>::begin(sampling_tree_.begin());
  tree<Node>::iterator end_node = tree<Node>::end(sampling_tree_.end());

  do {
    int node_depth = tree<Node>::depth(start_node);

    for (int i = 0; i < node_depth; ++i) {
      std::cout << " ";
    }

    std::cout << start_node->public_name_ << std::endl;

    start_node++;
  } while (start_node != end_node);
}

void TreeManager::grow_tree() {
  for (int i = 0; i < tree_target_depth_; ++i) {
    add_depth_level(i);
		eval_depth_level();
  }
}

void TreeManager::add_depth_level(size_t horizon_step) {
	t_ += config_.step_size;
	for (int leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos) {
		futures_[leaf_pos] = pool_->enqueue(bind(&TreeManager::add_node,this, std::placeholders::_1, std::placeholders::_2), horizon_step, leaf_pos);
	}

	//wait untill all threads have finished processing the tree depth
	for (size_t leaf_pos = 0; leaf_pos < tree_width_; leaf_pos++){
		leaf_handles_[leaf_pos] = futures_[leaf_pos].get();
	}

	tree_dynamics_v = tree_dynamics_next_v;
}

tree<Node>::iterator TreeManager::add_node(size_t horizon_step, size_t leaf_pos) {
	size_t expert_type;

	size_t extending_leaf_pos;
	tree<Node>::iterator extending_leaf;

	dynamics_ptr extending_dynamics;

	// treat first rollout differently
	if (leaf_pos == 0){
		// decide on experts
		expert_type = rollout_expert_map_[leaf_pos];

		// decide on extending leaf
		extending_leaf_pos = leaf_pos;
		extending_leaf = leaf_handles_[extending_leaf_pos];

		// what to do when leaf is extensible
	} else if (std::count(extendable_leaf_pos_.begin(), extendable_leaf_pos_.end(), leaf_pos)){
		// decide on experts
		expert_type = rollout_expert_map_[leaf_pos];

		// decide on extending leaf
		extending_leaf_pos = leaf_pos;
		extending_leaf = leaf_handles_[extending_leaf_pos];

		// what to do when leaf is NOT extensible
	} else {
		// decide on experts
		expert_type = rollout_expert_map_[leaf_pos];

		// decide on extending leaf
		size_t size_extensible_set = extendable_leaf_pos_.size();
		extending_leaf_pos = extendable_leaf_pos_[random_uniform_int(0,size_extensible_set)];
		extending_leaf = leaf_handles_[extending_leaf_pos];
	}

	extending_dynamics = tree_dynamics_v[extending_leaf_pos]->clone();
	Eigen::VectorXd u = expert_->get_sample(expert_type, horizon_step);
	Eigen::VectorXd x = extending_dynamics->step(u, config_.step_size);

	tree_dynamics_next_v[leaf_pos] = extending_dynamics;

	return sampling_tree_.append_child(extending_leaf, Node(extending_leaf->parent_node_,t_, config_, cost_, u, x));
}

void TreeManager::eval_depth_level(){
	extendable_leaf_pos_ = {};
	for (int leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos) {
		auto active_leaf = leaf_handles_[leaf_pos];

		if (active_leaf->c_ <10000000){
			extendable_leaf_pos_.push_back(leaf_pos);
		}
	}
};

void TreeManager::init_threading(size_t num_threads) {
  pool_ = std::make_unique<ThreadPool>(num_threads);
}

void TreeManager::time_it(){
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start_time_).count();
  std::cout << "Time since tree initialization: " << duration << "[μs], " << duration/1000000.0 << "[s], " << 1/(duration/1000000.0) << "[Hz]" << std::endl;
}

void TreeManager::transform_to_rollouts(){
  for (auto leaf_handle : leaf_handles_){
    auto path_to_leaf = sampling_tree_.path_from_iterator(leaf_handle, sampling_tree_.begin());

    for (size_t i = 0; i < path_to_leaf.size(); ++i ){
      std::vector<int> path_to_leaf_cut(path_to_leaf.begin(), path_to_leaf.begin()+i);
      auto value = sampling_tree_.iterator_from_path(path_to_leaf_cut, sampling_tree_.begin())->public_name_;
    }
  }
}

void TreeManager::reset(){
  sampling_tree_.clear();
  t_ = 0;
}

void TreeManager::build_new_tree(std::vector<dynamics_ptr> tree_dynamics_v, const observation_t x0_internal) {
	this->tree_dynamics_v = tree_dynamics_v;

  reset();
  init_tree(x0_internal );
  grow_tree();
  transform_to_rollouts();
}

int TreeManager::random_uniform_int(int v_min, int v_max){
	static std::mt19937 gen{ std::random_device{}() };
	std::uniform_int_distribution<int> u(v_min, v_max);
	return u(gen);
}

double TreeManager::random_uniform_double(double v_min, double v_max){
	static std::mt19937 gen{ std::random_device{}() };
	std::uniform_real_distribution<double> u(v_min, v_max);
	return u(gen);
}

void TreeManager::set_rollout_expert_mapping(size_t mapping_type_input){
  size_t mapping_type = 0;

  assert(config_.expert_types.size() == config_.expert_weights.size());

  // calculating the cumulative, normalized weights
  double weight_sum = 0;
  for (double weight : config_.expert_weights){
    weight_sum += weight;
  }
  double weight_cumul = 0;
  std::vector<double> normalized_expert_weight_cumul = {};
  for (double weight : config_.expert_weights){
    weight_cumul += weight/weight_sum;
    normalized_expert_weight_cumul.push_back(weight_cumul);
  }

  for (size_t rollout = 0; rollout < config_.rollouts; ++rollout){
    size_t expert_type = 0;

    double pos_rollout = (double)rollout / (double)config_.rollouts;

    switch (mapping_type) {
      // deterministic case (mapping is based on rollout number)
      case 0:
        for (int i = 0; i < config_.expert_weights.size(); ++i) {
          if (i == 0) {
            if (pos_rollout < normalized_expert_weight_cumul[i]) {
              expert_type = config_.expert_types[i];
              break;
            }
          } else {
            if (normalized_expert_weight_cumul[i - 1] <= pos_rollout & pos_rollout < normalized_expert_weight_cumul[i]) {
              expert_type = config_.expert_types[i];
              break;
            } else {
              std::cout << "This should not happen. If it does make special case if i==end" << std::endl;
              assert(1==0);
            }
          }
        }
        break;

      // probabilistic case (mapping is based on sampling)
      case 1:
        perror("this type of expert sampling is not yet implemented");
        assert(0==1);
        break;

      default:
        perror("please set mapping_type correctly!");
        assert(0==1);
        break;
    }

    rollout_expert_map_[rollout] = expert_type;
  }

}
