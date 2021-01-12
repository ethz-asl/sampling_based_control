/*!
 * @file     tree_manager.cpp
 * @author   Etienne Walther
 * @date     08.12.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi/tree/tree_manager.h"


#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wc++20-extensions"
TreeManager::TreeManager(const dynamics_ptr& dynamics, cost_ptr cost, sampler_ptr sampler, config_t config, mppi::Expert *expert) : sampling_tree_(), datalogger_timing_("/home/etienne/Documents/logging_directory/test/tree_timing_logger.txt"){
  cost_ = std::move(cost);
  dynamics_ = dynamics;
  config_ = std::move(config);
  sampler_ = std::move(sampler);
  expert_ = static_cast<std::shared_ptr<mppi::Expert>>(expert);

  set_rollout_expert_mapping(0);

  init_threading();

  datalogger_timing_.write("high_precision_timestamp",  "t0_internal", "horizonstep","description", "category");
  datalogger_timing_.write_endl();
}

void TreeManager::init_threading() {
		std::cout << "Using multithreading. Number of threads: " << config_.threads
							<< std::endl;
		pool_ = std::make_unique<ThreadPool>(config_.threads);
}

void TreeManager::build_new_tree(const std::vector<dynamics_ptr>& tree_dynamics_v, const observation_t& x0_internal, double t0_internal, const mppi::Rollout& opt_roll) {
  x0_internal_ = x0_internal;
  t0_internal_ = t0_internal;
  opt_roll_ = opt_roll;

  if (config_.log_tree_timing) {
    datalogger_timing_.write(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count(), t0_internal_, 0, "start of build new tree",
                             "build_new_tree");
    datalogger_timing_.write_endl();
  }

  this->tree_dynamics_v_ = tree_dynamics_v;
	this->tree_dynamics_v_shared_.resize(config_.rollouts);

	for (int i = 0; i < config_.rollouts; ++i) {
		this->tree_dynamics_v_shared_[i] = this->tree_dynamics_v_[i]->clone();
	}

  experts_v_.resize(config_.rollouts);
	for (int i = 0; i < config_.rollouts; ++i){
    experts_v_[i] = expert_->clone();
	}

  if (config_.log_tree_timing) {
    datalogger_timing_.write(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count(), t0_internal_, 0, "start of build new tree after reset dynamics",
                             "build_new_tree");
    datalogger_timing_.write_endl();
  }

	init_tree();

  if (config_.log_tree_timing) {
    datalogger_timing_.write(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count(), t0_internal_, 0, "start of build new tree after init tree",
                             "build_new_tree");
    datalogger_timing_.write_endl();
  }

	grow_tree();

  if (config_.log_tree_timing) {
    datalogger_timing_.write(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count(), t0_internal_, 0, "start of build new tree after grow tree",
                             "build_new_tree");
    datalogger_timing_.write_endl();
  }

	if (config_.debug_print){
		print_tree();
	}

	transform_to_rollouts();

  if (config_.log_tree_timing) {
    datalogger_timing_.write(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count(), t0_internal_, 0, "start of build new tree after convert to rollouts",
                             "build_new_tree");
    datalogger_timing_.write_endl();
  }
	time_it();
}

void TreeManager::init_tree(){
  start_time_ = std::chrono::high_resolution_clock::now();
  tree_width_ = config_.rollouts;
	tree_target_depth_ = std::floor(config_.horizon / config_.step_size);
	t_ = t0_internal_;

	sampling_tree_.clear();
	leaf_handles_.resize(tree_width_);
	extendable_leaf_pos_.clear();
	futures_.resize(tree_width_);

  rollouts_.clear();
	rollouts_.resize(config_.rollouts, mppi::Rollout(tree_target_depth_, dynamics_->get_input_dimension(), dynamics_->get_state_dimension()));

	rollouts_cost_ = Eigen::ArrayXd::Zero(config_.rollouts);

	// add root
  tree<Node>::iterator root = sampling_tree_.insert(sampling_tree_.begin(), Node(0, nullptr, 0, config_, cost_, dynamics_->get_zero_input(x0_internal_), x0_internal_,Eigen::MatrixXd::Identity(dynamics_->get_input_dimension(), dynamics_->get_input_dimension()),0));

  // add initial nodes

  for (int leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos) {
    leaf_handles_[leaf_pos] = sampling_tree_.append_child(root, Node(0, root, t_, config_, cost_, dynamics_->get_zero_input(x0_internal_), x0_internal_,Eigen::MatrixXd::Identity(dynamics_->get_input_dimension(), dynamics_->get_input_dimension()),0));
  }

  // make sure all initial nodes are extended in first iteration
	for (size_t leaf_pos=0; leaf_pos < tree_width_; ++leaf_pos){
		extendable_leaf_pos_.push_back(leaf_pos);
	}
}

void TreeManager::grow_tree() {
	for (int i = 1; i <= tree_target_depth_; ++i) {
//		std::cout << "adding depth level: " << std::to_string(i) <<std::endl;
    if (config_.log_tree_timing) {
      datalogger_timing_.write(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count(), t0_internal_, i, "before add_depth_level(i)",
                               "grow_tree");
      datalogger_timing_.write_endl();
    }

		add_depth_level(i);

    if (config_.log_tree_timing) {
      datalogger_timing_.write(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count(), t0_internal_, i,
                               "after add_depth_level(i) / before eval_depth_level()", "grow_tree");
      datalogger_timing_.write_endl();
    }
//		std::cout << "evaluating depth level: " << std::to_string(i) <<std::endl;
		eval_depth_level();

    if (config_.log_tree_timing) {
      datalogger_timing_.write(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count(), t0_internal_, i, "after eval_depth_level()",
                               "grow_tree");
      datalogger_timing_.write_endl();
    }
	}
//	std::cout << "tree is fully grown!" << std::endl;
}

void TreeManager::add_depth_level(size_t horizon_step) {
	t_ = t0_internal_ + (horizon_step * config_.step_size);

	futures_.clear();
	futures_.resize(tree_width_);

  if (config_.log_tree_timing) {
    datalogger_timing_.write(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count(), t0_internal_, horizon_step, "before parallel processing of nodes",
                             "add_depth_level");
    datalogger_timing_.write_endl();
  }

	// START parallel region
	for (int leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos) {
		futures_[leaf_pos] = pool_->enqueue(bind(&TreeManager::add_node, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4), horizon_step, leaf_pos, tree_dynamics_v_shared_[leaf_pos], experts_v_[leaf_pos]);
	}

	//wait until all threads have finished processing the tree depth
	for (size_t leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos){
		futures_[leaf_pos].wait();
	}
	// END parallel region

  if (config_.log_tree_timing) {
    datalogger_timing_.write(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count(), t0_internal_, horizon_step, "after parallel processing of nodes / before resetting dynamicws",
                             "add_depth_level");
    datalogger_timing_.write_endl();
  }

	// as soon as all leafs are processed transfer new leafs
	for (size_t leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos){
		leaf_handles_[leaf_pos] = futures_[leaf_pos].get();
		tree_dynamics_v_[leaf_pos]->reset(tree_dynamics_v_shared_[leaf_pos]->get_state());
	}

  if (config_.log_tree_timing) {
    datalogger_timing_.write(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count(), t0_internal_, horizon_step, "after resetting dynamics",
                             "add_depth_level");
    datalogger_timing_.write_endl();
  }
}

tree<Node>::iterator TreeManager::add_node(size_t horizon_step, size_t leaf_pos, dynamics_ptr node_dynamics, expert_ptr node_expert) {
	size_t expert_type;
	size_t extending_leaf_pos;
	tree<Node>::iterator extending_leaf;

  if (config_.log_tree_timing) {
    datalogger_timing_.write(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count(), t0_internal_, horizon_step, "beginning add_node",
                             "add_node");
    datalogger_timing_.write_endl();
  }

//	// printing stuff
//	std::stringstream ss;
//	ss << std::this_thread::get_id();
//	std::string thread_id = ss.str();
//
//	auto string_output = "thread: "+ thread_id + "  horizon step: " +  std::to_string(horizon_step) + "  rollout: " + std::to_string(leaf_pos);
//	std::cout << string_output << std::endl;
//	// printing stuff end

	// decide on experts
	expert_type = rollout_expert_map_[leaf_pos];

	if (leaf_pos == 0 || std::count(extendable_leaf_pos_.begin(), extendable_leaf_pos_.end(), leaf_pos)){
		// decide on extending leaf
		extending_leaf_pos = leaf_pos;
		extending_leaf = leaf_handles_[extending_leaf_pos];

	} else {
		// decide on extending leaf
		extending_leaf_pos = extendable_leaf_pos_[random_uniform_int(0,extendable_leaf_pos_.size()-1)];
		extending_leaf = leaf_handles_[extending_leaf_pos];
	}



	node_dynamics->reset(tree_dynamics_v_[extending_leaf_pos]->get_state());
//	extending_dynamics->reset(tree_dynamics_v_[extending_leaf_pos]->get_state());

  if (config_.log_tree_timing) {
    datalogger_timing_.write(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count(), t0_internal_, horizon_step, "after node_dynamics->reset.",
                             "add_node");
    datalogger_timing_.write_endl();
  }


	Eigen::VectorXd u;
	Eigen::MatrixXd sigma_inv;
	if (leaf_pos==0){
		u = opt_roll_.uu[horizon_step-1];

		expert_type = 1; // use importance sampler expert for sigma_inv
		sigma_inv = sigma_inv = node_expert->get_sigma_inv(expert_type, horizon_step-1);
	}
	else {
		u = node_expert->get_sample(expert_type, horizon_step-1);

		sigma_inv = node_expert->get_sigma_inv(expert_type, horizon_step-1);
	}

  if (config_.log_tree_timing) {
    datalogger_timing_.write(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count(), t0_internal_, horizon_step, "after getting samples and sigma inv.",
                             "add_node");
    datalogger_timing_.write_endl();
  }

	u = bound_input(u);

	node_dynamics->step(u, config_.step_size);

  if (config_.log_tree_timing) {
    datalogger_timing_.write(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count(), t0_internal_, horizon_step, "after stepping dynamics.",
                             "add_node");
    datalogger_timing_.write_endl();
  }

	Eigen::VectorXd x = node_dynamics->get_state();
	Eigen::VectorXd u_applied = u;

	tree_dynamics_v_shared_[leaf_pos] = node_dynamics;

	std::unique_lock<std::shared_mutex> lock(tree_mutex_);
	tree<Node>::iterator child_handle = sampling_tree_.append_child(extending_leaf, Node(horizon_step, extending_leaf, t_, config_, cost_, u_applied, x, sigma_inv, expert_type));
	lock.unlock();

  if (config_.log_tree_timing) {
    datalogger_timing_.write(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count(), t0_internal_, horizon_step, "after appending new node to tree.",
                             "add_node");
    datalogger_timing_.write_endl();
  }

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

void TreeManager::time_it(){
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start_time_).count();
  std::cout << "Time since tree initialization: " << duration << "[μs], " << duration/1000000.0 << "[s], " << 1/(duration/1000000.0) << "[Hz]" << std::endl;
}

void TreeManager::transform_to_rollouts(){

  for (size_t k = 0; k < tree_width_; ++k){
    auto path_to_leaf = sampling_tree_.path_from_iterator(leaf_handles_[k], sampling_tree_.begin());

//		std::cout << path_to_leaf.size() << std::endl;
//		for (int i = 0; i < path_to_leaf.size(); ++i) {
//			std::cout << path_to_leaf[i];
//			if (i!=path_to_leaf.size()-1){
//				std::cout <<  "-";
//			}
//		}
//		std::cout << std::endl;

    for (size_t horizon_step = 0; horizon_step < tree_target_depth_; ++horizon_step) {

    	std::vector<int> path_to_leaf_cut_current(path_to_leaf.begin(), path_to_leaf.begin() + horizon_step + 3);

//			std::cout << path_to_leaf_cut_current.size() << std::endl;
//			for (int i = 0; i < path_to_leaf_cut_current.size(); ++i) {
//				std::cout << path_to_leaf_cut_current[i];
//				if (i!=path_to_leaf_cut_current.size()-1){
//					std::cout <<  "-";
//				}
//			}
//			std::cout << std::endl;
//
//			std::cout << path_to_leaf_cut_next.size() << std::endl;
//			for (int i = 0; i < path_to_leaf_cut_next.size(); ++i) {
//				std::cout << path_to_leaf_cut_next[i];
//				if (i!=path_to_leaf_cut_next.size()-1){
//					std::cout <<  "-";
//				}
//			}
//			std::cout << std::endl;


			auto current_node = sampling_tree_.iterator_from_path(path_to_leaf_cut_current, sampling_tree_.begin());
			// recompute noise

			auto t = current_node->t_;
			auto xx = current_node->xx_;
			auto c = current_node->c_discounted;
			auto uu = opt_roll_.uu[horizon_step];

			Eigen::MatrixXd nn = current_node->uu_applied_ - opt_roll_.uu[horizon_step];
			Eigen::MatrixXd sigma_inv = current_node->sigma_inv_;

//      std::cout<< "rollout: "<< k <<" node: " << horizon_step << " processed!"<<std::endl;

			if (std::isnan(c)) {
				std::cout << "COST IS NAN!" << std::endl;
				throw std::runtime_error("Something went wrong ... dynamics diverged?");
			}

			rollouts_[k].tt[horizon_step] = t;  // t0
			rollouts_[k].xx[horizon_step] = xx; // x0
			rollouts_[k].uu[horizon_step] = uu; //
			rollouts_[k].nn[horizon_step] = nn; //
			rollouts_[k].cc(horizon_step) = c; 	//
			rollouts_[k].total_cost += c -
																	config_.lambda * opt_roll_.uu[horizon_step].transpose() *
																		sigma_inv *	rollouts_[k].nn[horizon_step] +
																 	config_.lambda * opt_roll_.uu[horizon_step].transpose() *
																		sigma_inv * opt_roll_.uu[horizon_step];

//			if (horizon_step==0 & k==0){
//				std::cout << "opt_roll tt_0: " << opt_roll_.tt[0] << " node_time t0: " << current_node->t_<< " t0_internal: " << t0_internal_<< std::endl;
//			}
		}

//    for (size_t horizon_step_print = 0; horizon_step_print < tree_target_depth_; ++horizon_step_print){
//    	std::cout << "r-> hs: "<< horizon_step_print << " t: "<< rollouts_[k].tt[horizon_step_print] << " xx_0: "<< rollouts_[k].xx[horizon_step_print][0] << " uu_0: "<< rollouts_[k].uu[horizon_step_print][0] << " nn_0: "<< rollouts_[k].nn[horizon_step_print][0] << " c: "<<rollouts_[k].cc[horizon_step_print]<< std::endl;
//    }

		rollouts_cost_[k] = rollouts_[k].total_cost;
//    std::cout << rollouts_cost_[k] << std::endl;

//		std::unique_lock<std::shared_mutex> lock(tree_mutex_);
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

void TreeManager::set_rollout_expert_mapping(size_t mapping_type_input){
  size_t mapping_type = 0;


  // calculating the cumulative, normalized weights
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

    //TODO: Set this back to expert_type!
    rollout_expert_map_[rollout] = expert_type;
  }
}
#pragma clang diagnostic pop