/*!
 * @file     tree_manager.cpp
 * @author   Etienne Walther
 * @date     08.12.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi/tree/tree_manager.h"

TreeManager::TreeManager(cost_ptr cost, const dynamics_ptr& dynamics, config_t config, sampler_ptr sampler,  mppi::Expert *expert) : sampling_tree_(){
  cost_ = cost;
  dynamics_ = dynamics;
  config_ = config;
  sampler_ = sampler;
  expert_ = static_cast<std::shared_ptr<mppi::Expert>>(expert);

  set_rollout_expert_mapping(0);

  init_threading(config_.threads);

	tree_dynamics_v_.resize(config_.rollouts);
	tree_dynamics_v_next_.resize(config_.rollouts);
}

void TreeManager::build_new_tree(std::vector<dynamics_ptr> tree_dynamics_v, const observation_t x0_internal, double t0_internal, mppi::Rollout opt_roll) {
	this->tree_dynamics_v_ = tree_dynamics_v;
	this->tree_dynamics_v_next_ = tree_dynamics_v;

	x0_internal_ = x0_internal;
	t0_internal_ = t0_internal;
	opt_roll_ = opt_roll;

	sampling_tree_.clear();
	t_ = 0;

	init_tree();
	grow_tree();
	if (config_.debug_print){
		print_tree();
	}
	std::cout << "print_tree done" << std::endl;
	transform_to_rollouts();
}

void TreeManager::init_tree(){
	std::cout << "Init tree started" << std::endl;
  start_time_ = std::chrono::high_resolution_clock::now();
  tree_width_ = config_.rollouts;

  // TODO: use steps_ from mppi
  tree_target_depth_ = std::floor(config_.horizon / config_.step_size);
	rollouts_.resize(config_.rollouts, mppi::Rollout(tree_target_depth_, dynamics_->get_input_dimension(), dynamics_->get_state_dimension()));
	rollouts_cost_ = Eigen::ArrayXd::Zero(config_.rollouts);

	futures_.resize(tree_width_);
  futures_.resize(tree_width_);

  tree<Node>::iterator root = sampling_tree_.insert(sampling_tree_.begin(), Node(0, nullptr, t0_internal_, config_, cost_, dynamics_->get_zero_input(x0_internal_), x0_internal_));

  leaf_handles_.resize(tree_width_);

  extendable_leaf_pos_.clear();
  for (size_t leaf_pos=0; leaf_pos < tree_width_; ++leaf_pos){
		extendable_leaf_pos_.push_back(leaf_pos);
  }

  for (int leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos) {
    leaf_handles_[leaf_pos] = sampling_tree_.append_child(root, Node(0, root->parent_node_, t0_internal_, config_, cost_, dynamics_->get_zero_input(x0_internal_), x0_internal_));
  }
}

void TreeManager::grow_tree() {
	for (int i = 0; i < tree_target_depth_; ++i) {
		std::cout << "depth level adding started" << std::endl;
		add_depth_level(i);
		std::cout << "eval depth level adding started" << std::endl;
		eval_depth_level();
	}
	std::cout << "grow tree done" << std::endl;
}

void TreeManager::add_depth_level(size_t horizon_step) {
	t_ = t0_internal_ + horizon_step * config_.step_size;

	// START parallel region
	for (int leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos) {
		futures_[leaf_pos] = pool_->enqueue(bind(&TreeManager::add_node, this, std::placeholders::_1, std::placeholders::_2), horizon_step, leaf_pos);
	}

	//wait untill all threads have finished processing the tree depth
	for (size_t leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos){
		futures_[leaf_pos].wait();
	}
	// END parallel region

	// as soon as all leafs are processed transfer new leafs
	for (size_t leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos){
		leaf_handles_[leaf_pos] = futures_[leaf_pos].get();
	}

	for (size_t leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos) {
		tree_dynamics_v_[leaf_pos] = tree_dynamics_v_next_[leaf_pos]->clone();
	}
}

tree<Node>::iterator TreeManager::add_node(size_t horizon_step, size_t leaf_pos) {
	size_t expert_type;

	size_t extending_leaf_pos;
	tree<Node>::iterator extending_leaf;

	dynamics_ptr extending_dynamics;

	// decide on experts
	expert_type = rollout_expert_map_[leaf_pos];

	if (leaf_pos == 0 || std::count(extendable_leaf_pos_.begin(), extendable_leaf_pos_.end(), leaf_pos)){
		// decide on extending leaf
		extending_leaf_pos = leaf_pos;
		extending_leaf = leaf_handles_[extending_leaf_pos];

	} else {
		// decide on extending leaf
		size_t size_extensible_set = extendable_leaf_pos_.size();
		extending_leaf_pos = extendable_leaf_pos_[random_uniform_int(0,size_extensible_set-1)];
		extending_leaf = leaf_handles_[extending_leaf_pos];
	}

	extending_dynamics = tree_dynamics_v_[extending_leaf_pos]->clone();

	Eigen::VectorXd u;
	if (leaf_pos==0){
		u = opt_roll_.uu[horizon_step];
	}	else {
		u = expert_->get_sample(expert_type, horizon_step);
	}

	Eigen::VectorXd x = extending_dynamics->step(u, config_.step_size);

	tree_dynamics_v_next_[leaf_pos] = extending_dynamics;

	return sampling_tree_.append_child(extending_leaf, Node(horizon_step, extending_leaf->parent_node_, t_, config_, cost_, u, x));
}

void TreeManager::eval_depth_level(){
	extendable_leaf_pos_.clear();
	double depth_min_cost = std::numeric_limits<double>::max();

	for (int rollout = 0; rollout < config_.rollouts; ++rollout) {
		auto active_rollout = leaf_handles_[rollout];
		if (active_rollout->c_ < depth_min_cost) {
			depth_min_cost = active_rollout->c_;
		}
	}

	for (int leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos) {
		auto active_leaf = leaf_handles_[leaf_pos];

		if (active_leaf->c_ <= depth_min_cost + (config_.pruning_threshold * depth_min_cost)){
			extendable_leaf_pos_.push_back(leaf_pos);
		}
	}
}

void TreeManager::print_tree() {
  tree<Node>::iterator start_node = tree<Node>::begin(sampling_tree_.begin());
  tree<Node>::iterator end_node = tree<Node>::end(sampling_tree_.end());

  do {
    int node_depth = tree<Node>::depth(start_node);

    for (int i = 0; i < node_depth; ++i) {
			if (i == node_depth - 1) {
				std::cout << "> ";
			} else {
				std::cout << "| ";
			}
    }

    std::cout << start_node->public_name_ << std::endl;

    start_node++;
  } while (start_node != end_node);
  std::cout << std::endl;
}

void TreeManager::init_threading(size_t num_threads) {
  pool_ = std::make_unique<ThreadPool>(num_threads);
}

void TreeManager::time_it(){
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start_time_).count();
  std::cout << "Time since tree initialization: " << duration << "[μs], " << duration/1000000.0 << "[s], " << 1/(duration/1000000.0) << "[Hz]" << std::endl;
}

void TreeManager::transform_to_rollouts(){

	std::cout << "Tranform to rollouts started" << std::endl;

  for (size_t k = 0; k < tree_width_; ++k){
    auto path_to_leaf = sampling_tree_.path_from_iterator(leaf_handles_[k], sampling_tree_.begin());

		size_t t = 0;
		std::cout << path_to_leaf.size() << std::endl;
		for (int i = 0; i < path_to_leaf.size(); ++i) {
			std::cout << path_to_leaf[i];
			if (i!=path_to_leaf.size()-1){
				std::cout <<  "-";
			}
		}
		std::cout << std::endl;
    for (size_t node_depth = 2; node_depth < path_to_leaf.size(); ++node_depth ) {
    	std::vector<int> path_to_leaf_cut(path_to_leaf.begin(), path_to_leaf.begin() + node_depth);
			auto current_node = sampling_tree_.iterator_from_path(path_to_leaf_cut, sampling_tree_.begin());

			// recompute noise
      auto nn = current_node->uu_ - opt_roll_.uu[t];
      auto uu = opt_roll_.uu[t];
      auto xx = current_node->xx_;
      auto c = current_node->c_discounted;

			rollouts_[k].xx[t] = xx;
			rollouts_[k].uu[t] = uu;
			rollouts_[k].nn[t] = nn;
			rollouts_[k].cc(t) = c;
			rollouts_[k].total_cost += c ;
			t++;
		}
		rollouts_cost_[k] = rollouts_[k].total_cost;
  }
	std::cout << "Tranform to rollouts done" << std::endl;
}

std::vector<mppi::Rollout> TreeManager::get_rollouts(){
	return rollouts_;
}

Eigen::ArrayXd TreeManager::get_rollouts_cost(){
	return rollouts_cost_;
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