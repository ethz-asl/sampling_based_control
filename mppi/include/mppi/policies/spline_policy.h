//
// Created by giuseppe on 25.05.21.
//

#pragma once

#include "mppi/core/policy.h"
#include "mppi/policies/receding_horizon_spline.h"

namespace mppi {
class SplinePolicy : public Policy {
 public:
  SplinePolicy(int nu);

  Eigen::VectorXd nominal(double t) override;

  Eigen::VectorXd sample(double t, int k) override;

  void update_samples(const std::vector<double>& weights, const int keep) override;

  void update(const std::vector<double>& weights, const double step_size) override;

  void shift(const double t) override;

  void bound() override {}; // TODO(giuseppe)
 private:
  std::vector<RecedingHorizonSpline> policies_;
};
}  // namespace mppi
