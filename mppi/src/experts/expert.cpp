/*!
 * @file     expert.cpp
 * @author   Etienne Walther
 * @date     08.12.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi/experts/expert.h"

namespace mppi {

Expert::Expert(config_t config, const dynamics_ptr& dynamics) {
  config_ = config;
  dynamics_ = dynamics;

  assert(config_.expert_types.size() == config_.expert_weights.size());

  for (auto& expert_type : config_.expert_types) {
    switch (expert_type) {
      case 0:
        experts_[expert_type] = new NormExp('N', config_, dynamics_);
      case 1:
        experts_[expert_type] = new ImpExp('I', config_, dynamics_);
    }
  }
}

Eigen::VectorXd Expert::get_sample(size_t expert_type, size_t step) {
  return experts_[expert_type]->get_sample(step);
}

Eigen::MatrixXd Expert::get_sigma_inv(size_t expert_type, size_t step) {
  return experts_[expert_type]->get_sigma_inv(step);
}

void Expert::update_expert(size_t expert_type,
                           const std::vector<Eigen::VectorXd>& mean) {
  switch (expert_type) {
    case 0:
      std::cout << "This expert can not be updated" << std::endl;
      assert(1 == 0);
    case 1:
      experts_[expert_type]->update_expert(mean);

    default:
      assert(0 == 1);
  }
}

}  // namespace mppi