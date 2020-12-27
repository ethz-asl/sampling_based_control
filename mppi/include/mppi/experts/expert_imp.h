//
// Created by etienne on 14.12.20.
//

#pragma once

#include "expert_base.h"

class ImpExp : public ExpertBase{
 public:
  ImpExp(char short_name, config_t config, dynamics_ptr dynamics): ExpertBase(short_name, config, dynamics), expert_sampler_one_(dynamics_->get_input_dimension()){

    expert_sampler_one_.set_covariance(std::vector<double> {1,1,1,1,1,1,1,1,1,1});
    expert_sampler_one_.set_mean(std::vector<double> {0,0,0,0,0,0,0,0,0,0});

    for (int i = 0; i < std::floor(config_.horizon / config_.step_size); ++i) {
			expert_sampler_map_[i] = expert_sampler_one_;
    }
  };

  ~ImpExp() = default;

  Eigen::VectorXd get_sample(size_t step) override {
    return expert_sampler_map_[step].get_sample();
  };

	Eigen::MatrixXd get_sigma(size_t step) override{
		return expert_sampler_map_[step].sigma();
	}

	Eigen::MatrixXd get_sigma_inv(size_t step) override{
		return expert_sampler_map_[step].sigma_inv();
	}

  void update_expert(std::vector<Eigen::VectorXd> mean) override {
    for (int step = 0; step < mean.size(); ++step) {
      expert_sampler_map_[step].set_mean(mean[step]);
    }
  };

 protected:
  mppi::GaussianSampler expert_sampler_one_;
  std::map<size_t, mppi::GaussianSampler> expert_sampler_map_;
};