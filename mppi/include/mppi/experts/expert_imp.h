//
// Created by etienne on 14.12.20.
//

#pragma once

#include "expert_base.h"


class ImpExp : public ExpertBase{
 public:
  ImpExp(char short_name, config_t config, dynamics_ptr dynamics): ExpertBase(short_name, config, dynamics), expert_sampler_one_(dynamics_->get_input_dimension()){

    expert_sampler_one_.set_covariance(std::vector<double> {2,2});
    expert_sampler_one_.set_mean(std::vector<double> {0,0});

    for (int i = 0; i < config_.rollouts; ++i) {
			expert_sampler_map_[i] = expert_sampler_one_;
    }
  };

  ~ImpExp() = default;

  Eigen::MatrixXd get_sample(size_t step) override {
    return expert_sampler_map_[step].get_sample();
  };

  void update_expert(Eigen::MatrixXd mean) override {
    for (int step = 0; step < mean.cols(); ++step) {
      expert_sampler_map_.at(step).set_mean(std::vector<double> {mean(0, step), mean(1, step)});
    }
    // calc exp weights etc
  };

 protected:
  mppi::GaussianSampler expert_sampler_one_;
  std::map<size_t, mppi::GaussianSampler> expert_sampler_map_;
};