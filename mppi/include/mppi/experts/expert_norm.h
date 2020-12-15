//
// Created by etienne on 14.12.20.
//

#pragma once

#include "expert_base.h"

class NormExp : public ExpertBase{
 public:
  NormExp(char short_name, config_t config, dynamics_ptr dynamics): ExpertBase(short_name, config, dynamics), expert_sampler_one_(dynamics_->get_input_dimension()){

    expert_sampler_one_.set_covariance(std::vector<double> {2,2});
    expert_sampler_one_.set_mean(std::vector<double> {0,0});

  };
  ~NormExp() = default;

  Eigen::MatrixXd get_sample(size_t step) override{
    return expert_sampler_one_.get_sample();
  }

  void update_expert(Eigen::MatrixXd mean) override{
    // do nothing
  }

 protected:
  mppi::GaussianSampler expert_sampler_one_;

};