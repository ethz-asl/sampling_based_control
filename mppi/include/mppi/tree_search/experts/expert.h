/*!
 * @file     expert.h
 * @author   Etienne Walther
 * @date     08.12.2020
 * @version  1.0
 * @brief    description
 */

#pragma once

#include <memory>

#include "mppi/core/config.h"
#include "mppi/core/dynamics.h"
#include "mppi/core/typedefs.h"
#include "mppi/tree_search/experts/expert_imp.h"
#include "mppi/tree_search/experts/expert_norm.h"

namespace mppi {

class Expert {
 public:
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