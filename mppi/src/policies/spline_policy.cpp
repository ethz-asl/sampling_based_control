//
// Created by giuseppe on 25.05.21.
//

#include "mppi/policies/spline_policy.h"

using namespace mppi;

// Spline Policy
SplinePolicy::SplinePolicy(int nu) : Policy(nu) {
  BSplinePolicyConfig cfg; // TODO(giuseppe) pass and initialize config properly
  policies_.resize(nu, RecedingHorizonSpline(cfg));
}

void SplinePolicy::shift(const double t) {
  for (auto& policy : policies_) policy.shift(t);
}

void SplinePolicy::update_samples(const std::vector<double> &weights, const int keep) {
  Eigen::VectorXd v = Eigen::VectorXd::Zero((int)weights.size()); // TODO this can be made more efficient
  for (int i=0; i<weights.size(); i++) v(i) = weights[i];
  for (auto policy : policies_)
    policy.update_samples(v, keep);
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
  for (int i=0; i<weights.size(); i++) v(i) = weights[i];
  for (auto & policy : policies_)
    policy.update(v, step_size);
}
