/*!
 * @file     expert.h
 * @author   Etienne Walther
 * @date     08.12.2020
 * @version  1.0
 * @brief    description
 */

#pragma once

#include <memory>

#include "expert_imp.h"
#include "expert_norm.h"
#include "mppi/dynamics/dynamics_base.h"
#include "mppi/solver_config.h"

namespace mppi {

class Expert {
 public:
  using expert_ptr = std::shared_ptr<Expert>;
  using config_t = mppi::SolverConfig;
  using dynamics_ptr = mppi::DynamicsBase::dynamics_ptr;

  Expert(config_t config, const dynamics_ptr& dynamics);
  ~Expert() = default;

  Eigen::VectorXd get_sample(size_t expert_type, size_t step);

  Eigen::MatrixXd get_sigma_inv(size_t expert_type, size_t step);

  void update_expert(size_t expert_type,
                     const std::vector<Eigen::VectorXd>& mean);

  expert_ptr clone() const { return std::make_shared<Expert>(*this); }

 private:
  std::map<size_t, int> rollout_expert_map;

  config_t config_;
  dynamics_ptr dynamics_;

  // create experts
  std::map<size_t, ExpertBase*> experts_;
};

}  // namespace mppi