//
// Created by etienne on 15.12.20.
//

#include "mppi/experts/expert.h"

namespace mppi {

// Expert
Expert::Expert(config_t config, const dynamics_ptr& dynamics) {
  // Define all the experts used
  config_ = config;
  dynamics_ = dynamics;

  experts_[0] = new NormExp('N', config_, dynamics_);
  experts_[1] = new ImpExp('I',config_, dynamics_);
}

Eigen::VectorXd Expert::get_sample(size_t expert_type, size_t step) {
  return experts_[expert_type]->get_sample(step);
}

void Expert::update_expert(size_t expert_type, Eigen::MatrixXd mean) {
  experts_[expert_type]->update_expert(mean);
}

void Expert::update_experts() {
  // do nothing
}

}