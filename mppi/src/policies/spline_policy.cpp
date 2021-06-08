//
// Created by giuseppe on 25.05.21.
//

#include "mppi/policies/spline_policy.h"

using namespace mppi;

// Spline Policy
// TODO(giuseppe) take params from config
SplinePolicy::SplinePolicy(int nu, const Config& config) : Policy(nu) {
  BSplinePolicyConfig cfg; // TODO(giuseppe) pass and initialize config properly
  cfg.horizon = config.horizon;
  cfg.dt = config.step_size;
  cfg.samples = config.rollouts;
  cfg.degree = 3;
  cfg.cp_dt = 0.15;
  cfg.verbose = false;

  policies_.clear();
  policies_.resize(nu_, RecedingHorizonSpline(cfg));

  for (int i = 0; i < nu; i++) {
    cfg.sigma = config.input_variance[i];
    cfg.max_value = config.u_max[i];
    cfg.min_value = config.u_min[i];
    policies_[i] = RecedingHorizonSpline(cfg);
  }
}

void SplinePolicy::shift(const double t) {
  for (auto& policy : policies_) {
    // std::cout << "Sample matrix: " << std::endl;
    // std::cout << policy.P_ << std::endl;
    // std::cout << "Nominal matrix: " << std::endl;
    // std::cout << policy.Pn_.transpose() << std::endl;
    // std::cout << "Shifting policies" << std::endl;
    policy.shift(t);
    // std::cout << "Sample matrix after shift: " << std::endl;
    // std::cout << policy.P_ << std::endl;
    // std::cout << "Nominal matrix after shift: " << std::endl;
    // std::cout << policy.Pn_.transpose() << std::endl;
  }
}

void SplinePolicy::update_samples(const std::vector<double> &weights, const int keep) {
  Eigen::VectorXd v = Eigen::VectorXd::Zero((int)weights.size()); // TODO this can be made more efficient
  for (int i=0; i<weights.size(); i++) v(i) = weights[i];

  for (auto& policy : policies_) {
    policy.update_samples(v, keep);
  }
}

Eigen::VectorXd SplinePolicy::sample(double t, int k) {
  int t_idx = policies_[0].get_time_idx(t);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(nu_);
  for (int i=0; i<policies_.size(); i++)
    v(i) = policies_[i].P_(t_idx, k);
  return v;
}

Eigen::VectorXd SplinePolicy::nominal(double t){
  int t_idx = policies_[0].get_time_idx(t);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(nu_);
  for (int i=0; i<policies_.size(); i++)
    v(i) = policies_[i].Pn_(t_idx);
  return v;
}

void SplinePolicy::update(const std::vector<double> &weights, const double step_size) {
  Eigen::VectorXd v = Eigen::VectorXd::Zero((int)weights.size());
  for (int i = 0; i < weights.size(); i++) v(i) = weights[i];

  // std::cout << "Weights are: " << v.transpose() << std::endl;
  for (auto& policy : policies_) {
    policy.update(v, step_size);
    // std::cout << "Sample matrix after update: " << std::endl;
    // std::cout << policy.P_ << std::endl;
    // std::cout << "Nominal matrix after update: " << std::endl;
    // std::cout << policy.Pn_.transpose() << std::endl;
  }
}
