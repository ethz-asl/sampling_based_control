//
// Created by etienne on 14.12.20.
//

#pragma once

#include "expert_base.h"

class NormExp : public ExpertBase{
 public:
  NormExp(char short_name, config_t config, dynamics_ptr dynamics): ExpertBase(short_name, config, dynamics), expert_sampler_one_(dynamics_->get_input_dimension()){
		expert_sampler_one_.set_covariance(Eigen::VectorXd::Ones(dynamics_->get_input_dimension())*0.05);
  }

  ~NormExp() = default;

  Eigen::VectorXd get_sample(size_t step) override{
    return expert_sampler_one_.get_sample();
  }

	Eigen::MatrixXd get_sigma_inv(size_t step) override{
		return expert_sampler_one_.sigma_inv();
	}

  void update_expert(std::vector<Eigen::VectorXd> mean) override{
    // do nothing
  }

 protected:
  mppi::GaussianSampler expert_sampler_one_;

};