/*!
 * @file     tree_manager.cpp
 * @author   Etienne Walther
 * @date     08.12.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi/tree/tree_manager.h"

TreeManager::TreeManager(const dynamics_ptr& dynamics, cost_ptr cost, sampler_ptr sampler, config_t config, mppi::Expert *expert) : sampling_tree_(){
  cost_ = std::move(cost);
  dynamics_ = dynamics;
  config_ = std::move(config);
  sampler_ = std::move(sampler);
  expert_ = static_cast<std::shared_ptr<mppi::Expert>>(expert);

  gen_rollout_expert_mapping(0);
  init_threading();
}

void TreeManager::init_threading() {
		std::cout << "Using multithreading. Number of threads: " << config_.threads << std::endl;
		pool_ = std::make_unique<ThreadPool>(config_.threads);
		pool_add_leaf_ = std::make_unique<ThreadPool>(1);
}

void TreeManager::build_new_tree(const std::vector<dynamics_ptr>& tree_dynamics_v, const observation_t& x0_internal, double t0_internal, const mppi::Rollout& opt_roll) {
  x0_internal_ = x0_internal;
  t0_internal_ = t0_internal;
  opt_roll_ = opt_roll;

  this->tree_dynamics_v_ = tree_dynamics_v;
	this->tree_dynamics_v_shared_.resize(config_.rollouts);
	for (int i = 0; i < config_.rollouts; ++i) {
		this->tree_dynamics_v_shared_[i] = tree_dynamics_v_[i]->clone();
	}

  experts_v_.resize(config_.rollouts);
	for (int i = 0; i < config_.rollouts; ++i){
    experts_v_[i] = expert_->clone();
	}

	init_tree();
	grow_tree();

	if (config_.debug_print){
		print_tree();
	}

	transform_to_rollouts();
}

void TreeManager::init_tree(){
  tree_width_ = config_.rollouts;
	tree_target_depth_ = std::floor(config_.horizon / config_.step_size);
	t_ = t0_internal_;

	sampling_tree_.clear();
	leaf_handles_.resize(tree_width_);
	extendable_leaf_pos_.clear();

	futures_.resize(tree_width_);
  futures_add_leaf_to_tree_.resize(tree_width_);

  rollouts_.clear();
	rollouts_.resize(config_.rollouts, mppi::Rollout(tree_target_depth_, dynamics_->get_input_dimension(), dynamics_->get_state_dimension()));

	rollouts_cost_ = Eigen::ArrayXd::Zero(config_.rollouts);

  tree<Node>::iterator root = sampling_tree_.insert(sampling_tree_.begin(), Node(0, nullptr, 0, config_, cost_, dynamics_->get_zero_input(x0_internal_), x0_internal_,Eigen::MatrixXd::Identity(dynamics_->get_input_dimension(), dynamics_->get_input_dimension()),0));

  for (int leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos) {
    leaf_handles_[leaf_pos] = sampling_tree_.append_child(root, Node(0, root, t_, config_, cost_, dynamics_->get_zero_input(x0_internal_), x0_internal_,Eigen::MatrixXd::Identity(dynamics_->get_input_dimension(), dynamics_->get_input_dimension()),0));
  }

	for (size_t leaf_pos=0; leaf_pos < tree_width_; ++leaf_pos){
		extendable_leaf_pos_.push_back(leaf_pos);
	}
}

void TreeManager::grow_tree() {
	for (int horizon_step = 1; horizon_step <= tree_target_depth_; ++horizon_step) {
		add_depth_level(horizon_step);
		eval_depth_level();
	}
}

void TreeManager::add_depth_level(size_t horizon_step) {
	t_ = t0_internal_ + (horizon_step * config_.step_size);

	futures_.clear();
	futures_.resize(tree_width_);

  futures_add_leaf_to_tree_.clear();
  futures_add_leaf_to_tree_.resize(tree_width_);

	for (int leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos) {
		futures_[leaf_pos] = pool_->enqueue(bind(&TreeManager::add_node, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4), horizon_step, leaf_pos, tree_dynamics_v_shared_[leaf_pos], experts_v_[leaf_pos]);
	}

	for (size_t leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos){
		futures_[leaf_pos].wait();
    futures_add_leaf_to_tree_[leaf_pos].wait();
	}

	for (size_t leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos){
		leaf_handles_[leaf_pos] = futures_add_leaf_to_tree_[leaf_pos].get();
		tree_dynamics_v_[leaf_pos]->reset(tree_dynamics_v_shared_[leaf_pos]->get_state());
	}
}

bool TreeManager::add_node(size_t horizon_step, size_t leaf_pos, const dynamics_ptr& node_dynamics, const expert_ptr& node_expert) {
	size_t expert_type;
	size_t extending_leaf_pos;
	tree<Node>::iterator extending_leaf;

	expert_type = rollout_expert_map_[leaf_pos];

	if (leaf_pos == 0 || std::count(extendable_leaf_pos_.begin(), extendable_leaf_pos_.end(), leaf_pos)){
		extending_leaf_pos = leaf_pos;
		extending_leaf = leaf_handles_[extending_leaf_pos];

	} else {
		extending_leaf_pos = extendable_leaf_pos_[random_uniform_int(0,extendable_leaf_pos_.size()-1)];
		extending_leaf = leaf_handles_[extending_leaf_pos];
	}

	node_dynamics->reset(tree_dynamics_v_[extending_leaf_pos]->get_state());

	Eigen::VectorXd u;
	Eigen::MatrixXd sigma_inv;
	if (leaf_pos!=0){
    u = node_expert->get_sample(expert_type, horizon_step-1);
    sigma_inv = node_expert->get_sigma_inv(expert_type, horizon_step-1);
	}
	else {
    u = opt_roll_.uu[horizon_step-1];
    expert_type = 1;
    sigma_inv = node_expert->get_sigma_inv(expert_type, horizon_step-1);
	}

	u = bound_input(u);
	Eigen::VectorXd x = node_dynamics->step(u, config_.step_size);
	Eigen::VectorXd u_applied = u;

	tree_dynamics_v_shared_[leaf_pos] = node_dynamics;

  futures_add_leaf_to_tree_[leaf_pos] = pool_add_leaf_->enqueue(bind(&TreeManager::append_child_to_tree_via_pool, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6),
                                                                extending_leaf, horizon_step, u_applied, x, sigma_inv, expert_type);

	return true;
}

tree<Node>::iterator TreeManager::append_child_to_tree_via_pool(tree<Node>::iterator extending_leaf, size_t horizon_step, const Eigen::VectorXd& u_applied, const Eigen::VectorXd& x, const Eigen::MatrixXd& sigma_inv, size_t expert_type_applied){
  tree<Node>::iterator child_handle = sampling_tree_.append_child(extending_leaf, Node(horizon_step, extending_leaf, t_, config_, cost_, u_applied, x, sigma_inv, expert_type_applied));
  return child_handle;
}

void TreeManager::eval_depth_level(){
	extendable_leaf_pos_.clear();
	double depth_min_cost = std::numeric_limits<double>::max();

	for (int rollout = 0; rollout < config_.rollouts; ++rollout) {
		auto active_leaf = leaf_handles_[rollout];
		if (active_leaf->c_cum_ < depth_min_cost) {
			depth_min_cost = active_leaf->c_cum_;
		}
	}

	for (int leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos) {
		auto active_leaf = leaf_handles_[leaf_pos];
		if (active_leaf->c_cum_ <= depth_min_cost + (config_.pruning_threshold * depth_min_cost)){
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

void TreeManager::transform_to_rollouts(){
  for (size_t k = 0; k < tree_width_; ++k){
    auto path_to_leaf = sampling_tree_.path_from_iterator(leaf_handles_[k], sampling_tree_.begin());

    for (size_t horizon_step = 0; horizon_step < tree_target_depth_; ++horizon_step) {

    	std::vector<int> path_to_leaf_cut_current(path_to_leaf.begin(), path_to_leaf.begin() + horizon_step + 3);

			auto current_node = sampling_tree_.iterator_from_path(path_to_leaf_cut_current, sampling_tree_.begin());

			auto tt = current_node->t_;
			auto xx = current_node->xx_;
			auto c = current_node->c_discounted;
			auto uu = opt_roll_.uu[horizon_step];

			Eigen::MatrixXd nn = current_node->uu_applied_ - opt_roll_.uu[horizon_step];
			Eigen::MatrixXd sigma_inv = current_node->sigma_inv_;

			if (std::isnan(c)) {
				std::cout << "COST IS NAN!" << std::endl;
				throw std::runtime_error("Something went wrong ... dynamics diverged?");
			}

			rollouts_[k].tt[horizon_step] = tt;
			rollouts_[k].xx[horizon_step] = xx;
			rollouts_[k].uu[horizon_step] = uu;
			rollouts_[k].nn[horizon_step] = nn;
			rollouts_[k].cc(horizon_step) = c;
			rollouts_[k].total_cost += c -
																	config_.lambda * opt_roll_.uu[horizon_step].transpose() *
																		sigma_inv *	rollouts_[k].nn[horizon_step] +
																 	config_.lambda * opt_roll_.uu[horizon_step].transpose() *
																		sigma_inv * opt_roll_.uu[horizon_step];

		}

		rollouts_cost_[k] = rollouts_[k].total_cost;
  }
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

Eigen::VectorXd TreeManager::bound_input(input_t& u) {
  if (config_.bound_input){
    u = u.cwiseMax(config_.u_min).cwiseMin(config_.u_max);
  }
  return u;
}

void TreeManager::gen_rollout_expert_mapping(size_t mapping_type_input){
  size_t mapping_type = mapping_type_input;

  double weight_sum = 0;
  for (auto i=0; i<config_.expert_weights.size();++i){
    weight_sum += config_.expert_weights[i];
  }

  double weight_cumul = 0;
  std::vector<double> normalized_expert_weight_cumul = {};
	for (auto i=0; i<config_.expert_weights.size();++i){
    weight_cumul += config_.expert_weights[i]/weight_sum;
    normalized_expert_weight_cumul.push_back(weight_cumul);
  }

  for (size_t rollout = 0; rollout < config_.rollouts; ++rollout){
    size_t expert_type;

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
            }
          }
        }
        break;

      // different mapping type
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