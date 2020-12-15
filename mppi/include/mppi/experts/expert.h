//
// Created by etienne on 15.12.20.
//

#pragma once

#include <memory>

#include "mppi/solver_config.h"
#include "mppi/dynamics/dynamics_base.h"
#include "expert_imp.h"
#include "expert_norm.h"

namespace mppi {

class Expert {
 public:
  using expert_ptr = std::shared_ptr<Expert>;
	using config_t = mppi::SolverConfig;
	using dynamics_ptr = mppi::DynamicsBase::dynamics_ptr;
  Expert(config_t config, const dynamics_ptr& dynamics);
  ~Expert() = default;

  std::vector<int> expert_type_list_;

  int get_expert_from_LUT(size_t rollout);

  mppi::GaussianSampler get_expert_sampler(
      const std::vector<double>& state, size_t expert_type,
      const mppi::GaussianSampler& sampler_parent);

  Eigen::MatrixXd get_sample(size_t expert_type, size_t step);

  void update_expert(size_t expert_type, Eigen::MatrixXd mean);

  void update_experts();

 private:
  std::map<size_t, int> rollout_expert_map;

	config_t config_;
	dynamics_ptr dynamics_;

  // create experts
  std::map<size_t, ExpertBase*> experts_;
};

}