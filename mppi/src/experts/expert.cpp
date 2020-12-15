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

  experts_[0] = new NormExp('NORM', config_, dynamics_);
  experts_[1] = new ImpExp('IMP',config_, dynamics_);
}

mppi::GaussianSampler Expert::get_expert_sampler(
    const std::vector<double>& state, size_t expert_type,
    const mppi::GaussianSampler& sampler_parent) {
  // assert() that the expert is in the list of experts!

  mppi::GaussianSampler expert_sampler(dynamics_->get_input_dimension());

  switch (expert_type) {
    // Gauss random
    case 0:
      expert_sampler.set_covariance(std::vector<double>{2, 2});
      expert_sampler.set_mean(std::vector<double>{0, 0});

      break;
    case 1:
      expert_sampler.set_covariance(std::vector<double>{2, 2});
      expert_sampler.set_mean(std::vector<double>{0, 0});
      break;
      //		case 2:
      //			expert_sampler.set_covariance(std::vector<double> {10,10}); 			expert_sampler.set_mean(std::vector<double> {0,0}); 			break;
      //			// Gauss and informed by previous
      //		case 3:
      //			static std::mt19937 gen{ std::random_device{}() }; 			static std::normal_distribution<double> ann;
      //
      //			expert_sampler.set_covariance(std::vector<double> {2,2}); 			expert_sampler.set_mean(std::vector<double> {ann(gen),ann(gen)});
      //
      //			expert_sampler.combine_dist_mult(sampler_parent);
      //			break;
      //		case 4:
      //			expert_sampler.set_covariance(std::vector<double> {4,4}); 			expert_sampler.set_mean(std::vector<double> {0,0});
      //
      //			expert_sampler.combine_dist_mult(sampler_parent);
      //			break;
    default:
      expert_sampler.set_covariance(std::vector<double>{2, 2});
      expert_sampler.set_mean(std::vector<double>{0, 0});
      break;
  }

  return expert_sampler;
}

int Expert::get_expert_from_LUT(size_t rollout) {
  return rollout_expert_map.at(rollout);
}

Eigen::MatrixXd Expert::get_sample(size_t expert_type, size_t step) {
  return experts_[expert_type]->get_sample(step);
}

void Expert::update_expert(size_t expert_type, Eigen::MatrixXd mean) {
  experts_[expert_type]->update_expert(mean);
}

void Expert::update_experts() {
  // do nothing
}

}