/*!
 * @file     tree_manager.cpp
 * @author   Etienne Walther, Giuseppe Rizzi
 * @date     08.12.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi/tree_search/tree/tree_manager.h"
// #include <omp.h>

using namespace mppi;

TreeManager::TreeManager(const dynamics_ptr& dynamics, cost_ptr cost,
                         sampler_ptr sampler, config_t config,
                         mppi::Expert* expert)
    : sampling_tree_() {
  cost_ = std::move(cost);
  dynamics_ = dynamics;
  config_ = std::move(config);
  sampler_ = std::move(sampler);
  expert_ = static_cast<std::shared_ptr<mppi::Expert>>(expert);

  tree_width_ = config_.rollouts;
  tree_target_depth_ = std::floor(config_.horizon / config_.step_size);

  this->tree_dynamics_v_shared_.resize(config_.rollouts);
  this->cost_v_.resize(config_.rollouts);
  leaves_state_.resize(config_.rollouts);

  for (size_t i = 0; i < config_.rollouts; ++i) {
    this->tree_dynamics_v_shared_[i] = dynamics_->create();
    this->cost_v_[i] = cost_->create();
  }

  experts_v_.resize(config_.rollouts);
  for (size_t i = 0; i < config_.rollouts; ++i) {
    experts_v_[i] = expert_->clone();
  }

  rollouts_cost_.resize(config_.rollouts, 0.0);
  rollouts_.resize(
      config_.rollouts,
      mppi::Rollout(tree_target_depth_, dynamics_->get_input_dimension(),
                    dynamics_->get_state_dimension()));

  gen_rollout_expert_mapping(0);
  init_threading();

  leaf_handles_.resize(tree_width_);
  extensions_.resize(tree_width_);
}

void TreeManager::init_threading() {
  std::cout << "Using multithreading. Number of threads: " << config_.threads
            << std::endl;
  pool_ = std::make_unique<ThreadPool>(config_.threads);
  futures_.resize(tree_width_);
}

void TreeManager::set_reference_trajectory(
    const mppi::reference_trajectory_t& traj) {
  for (auto& cost : cost_v_) cost->set_reference_trajectory(traj);
}

void TreeManager::build_new_tree(const observation_t& x0_internal,
                                 double t0_internal,
                                 const mppi::Rollout& opt_roll) {
  x0_internal_ = x0_internal;
  t0_internal_ = t0_internal;
  opt_roll_ = opt_roll;

  init_tree();
  grow_tree();

  if (config_.debug_print) {
    print_tree();
  }

  transform_to_rollouts();
}

void TreeManager::init_tree() {
  t_ = t0_internal_;

  sampling_tree_.clear();
  extendable_leaf_pos_.clear();
  rollouts_cost_.resize(config_.rollouts, 0.0);

  node_iterator_t root = sampling_tree_.insert(
      sampling_tree_.begin(),
      node_t(nullptr, 0, 0, dynamics_->get_zero_input(x0_internal_),
             x0_internal_, 0));

  for (size_t leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos) {
    leaf_handles_[leaf_pos] = sampling_tree_.append_child(
        root, node_t(nullptr, 0, 0, dynamics_->get_zero_input(x0_internal_),
                     x0_internal_, 0));
  }

  for (size_t leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos) {
    extendable_leaf_pos_.push_back(leaf_pos);
  }
}

void TreeManager::grow_tree() {
  // reset to initial state first
  for (size_t i = 0; i < config_.rollouts; i++) {
    tree_dynamics_v_shared_[i]->reset(x0_internal_, t0_internal_);
    leaves_state_[i] = x0_internal_;
  }

  for (size_t horizon_step = 1; horizon_step <= tree_target_depth_;
       ++horizon_step) {
    add_depth_level(horizon_step);
    eval_depth_level();
  }
}

void TreeManager::add_depth_level(size_t horizon_step) {
  t_ = t0_internal_ + (horizon_step * config_.step_size);

  for (size_t leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos) {
    futures_[leaf_pos] =
        pool_->enqueue(bind(&TreeManager::add_node, this, std::placeholders::_1,
                            std::placeholders::_2, std::placeholders::_3,
                            std::placeholders::_4),
                       horizon_step, leaf_pos,
                       tree_dynamics_v_shared_[leaf_pos], experts_v_[leaf_pos]);
  }

  for (size_t leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos) {
    futures_[leaf_pos].wait();
  }
  // TODO(giuseppe) understand what makes omp so slow
  //  {
  //#pragma omp parallel for default(none) shared(horizon_step,
  // tree_dynamics_v_shared_, experts_v_)
  // ordered schedule(dynamic)
  //    for (int leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos){
  //      add_node(horizon_step, leaf_pos, tree_dynamics_v_shared_[leaf_pos],
  //      experts_v_[leaf_pos]);
  //    }
  //  }

  // tree is not thread-safe. Create new level after all nodes have been
  // appended
  for (size_t leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos) {
    leaf_handles_[leaf_pos] = sampling_tree_.append_child(
        extensions_[leaf_pos].first, extensions_[leaf_pos].second);
    leaves_state_[leaf_pos] = tree_dynamics_v_shared_[leaf_pos]->get_state();
  }
}

bool TreeManager::add_node(size_t horizon_step, size_t leaf_pos,
                           const dynamics_ptr& node_dynamics,
                           const expert_ptr& node_expert) {
  size_t expert_type;
  size_t extending_leaf_pos;
  node_iterator_t extending_leaf;

  expert_type = rollout_expert_map_[leaf_pos];

  if (leaf_pos == 0 || std::count(extendable_leaf_pos_.begin(),
                                  extendable_leaf_pos_.end(), leaf_pos)) {
    extending_leaf_pos = leaf_pos;
    extending_leaf = leaf_handles_[extending_leaf_pos];

  } else {
    extending_leaf_pos = extendable_leaf_pos_[random_uniform_int(
        0, extendable_leaf_pos_.size() - 1)];
    extending_leaf = leaf_handles_[extending_leaf_pos];

    node_dynamics->reset(leaves_state_[extending_leaf_pos], 0);
  }

  // TODO(giuseppe) all the time this stuff gets created
  Eigen::VectorXd u;
  if (leaf_pos != 0) {
    u = node_expert->get_sample(expert_type, horizon_step - 1);
  } else {
    u = opt_roll_.uu[horizon_step - 1];
    //    expert_type = 1;
  }

  bound_input(u);
  Eigen::VectorXd x = node_dynamics->step(u, config_.step_size);

  tree_dynamics_v_shared_[leaf_pos] = node_dynamics;

  double cost = cost_v_[leaf_pos]->get_stage_cost(x, u, t_) *
                std::pow(config_.discount_factor, horizon_step);
  extensions_[leaf_pos] = std::make_pair(
      extending_leaf, node_t(extending_leaf, t_, cost, u, x, expert_type));

  return true;
}

void TreeManager::eval_depth_level() {
  extendable_leaf_pos_.clear();
  double depth_min_cost = std::numeric_limits<double>::max();

  for (size_t rollout = 0; rollout < config_.rollouts; ++rollout) {
    if (leaf_handles_[rollout]->c_cum_ < depth_min_cost) {
      depth_min_cost = leaf_handles_[rollout]->c_cum_;
    }
  }

  const double pruning_threshold = 1.0;  // TODO HARD CODED
  for (size_t leaf_pos = 0; leaf_pos < tree_width_; ++leaf_pos) {
    auto& active_leaf = leaf_handles_[leaf_pos];
    if (active_leaf->c_cum_ <=
        depth_min_cost + (pruning_threshold * depth_min_cost)) {
      extendable_leaf_pos_.push_back(leaf_pos);
    }
  }
}

void TreeManager::print_tree() {
  auto start_node = tree<node_t>::begin(sampling_tree_.begin());
  auto end_node = tree<node_t>::end(sampling_tree_.end());

  do {
    int node_depth = tree<node_t>::depth(start_node);

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

void TreeManager::transform_to_rollouts() {
  // TODO(giuseppe) this can also be parallelized easily with omp
  for (size_t k = 0; k < tree_width_; ++k) {
    // extract path K of depth T (size path_to_leaf = T)
    auto path_to_leaf = sampling_tree_.path_from_iterator(
        leaf_handles_[k], sampling_tree_.begin());

    for (size_t horizon_step = 0; horizon_step < tree_target_depth_;
         ++horizon_step) {
      std::vector<int> path_to_leaf_cut_current(
          path_to_leaf.begin(), path_to_leaf.begin() + horizon_step + 3);

      auto current_node = sampling_tree_.iterator_from_path(
          path_to_leaf_cut_current, sampling_tree_.begin());

      if (std::isnan(current_node->c_)) {
        std::cout << "COST IS NAN!" << std::endl;
        throw std::runtime_error("Something went wrong ... dynamics diverged?");
      }

      const Eigen::MatrixXd& sigma_inv =
          experts_v_[k]->get_sigma_inv(rollout_expert_map_[k], horizon_step);
      rollouts_[k].tt[horizon_step] = current_node->t_;
      rollouts_[k].xx[horizon_step] = current_node->x_;
      rollouts_[k].uu[horizon_step] = current_node->u_;
      rollouts_[k].nn[horizon_step] =
          current_node->u_ - opt_roll_.uu[horizon_step];
      rollouts_[k].cc(horizon_step) = current_node->c_;
      rollouts_[k].total_cost += rollouts_[k].cc(horizon_step);  // -
      config_.lambda* opt_roll_.uu[horizon_step].transpose() *
              sigma_inv* rollouts_[k].nn[horizon_step] +
          config_.lambda* opt_roll_.uu[horizon_step].transpose() *
              sigma_inv* opt_roll_.uu[horizon_step];
    }

    rollouts_cost_[k] = rollouts_[k].total_cost;
  }
}

std::vector<mppi::Rollout> TreeManager::get_rollouts() { return rollouts_; }

std::vector<double> TreeManager::get_rollouts_cost() { return rollouts_cost_; }

int TreeManager::random_uniform_int(int v_min, int v_max) {
  static std::mt19937 gen{std::random_device{}()};
  std::uniform_int_distribution<int> u(v_min, v_max);
  return u(gen);
}

void TreeManager::bound_input(input_t& u) const {
  if (config_.bound_input) {
    u = u.cwiseMax(config_.u_min).cwiseMin(config_.u_max);
  };
}

void TreeManager::gen_rollout_expert_mapping(size_t mapping_type_input) {
  size_t mapping_type = mapping_type_input;

  double weight_sum = 0;
  static const int num_experts = 2;  // same weight to the 2 experts
  for (auto i = 0; i < num_experts; ++i) {
    weight_sum += 1;
  }

  double weight_cumul = 0;
  std::vector<double> normalized_expert_weight_cumul = {};
  for (auto i = 0; i < num_experts; ++i) {
    weight_cumul += 1.0 / weight_sum;
    normalized_expert_weight_cumul.push_back(weight_cumul);
  }

  for (size_t rollout = 0; rollout < config_.rollouts; ++rollout) {
    if (rollout == 0) {
      // TODO(giuseppe) use enums for this experts
      rollout_expert_map_[rollout] =
          1;  // first rollout always with Importance Sampling
      continue;
    }
    size_t expert_type;

    double pos_rollout = (double)rollout / (double)config_.rollouts;

    switch (mapping_type) {
      // deterministic case (mapping is based on rollout number)
      case 0:
        for (int i = 0; i < num_experts; ++i) {
          expert_type = i;
          if (i == 0) {
            if (pos_rollout < normalized_expert_weight_cumul[i]) {
              break;
            }
          } else {
            if ((normalized_expert_weight_cumul[i - 1] <= pos_rollout) &
                (pos_rollout < normalized_expert_weight_cumul[i])) {
              break;
            }
          }
        }
        break;

      // different mapping type
      case 1:
        perror("this type of expert sampling is not yet implemented");
        assert(0 == 1);
        break;

      default:
        perror("please set mapping_type correctly!");
        assert(0 == 1);
        break;
    }
    rollout_expert_map_[rollout] = expert_type;
  }
}