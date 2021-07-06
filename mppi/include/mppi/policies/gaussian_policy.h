//
// Created by giuseppe on 31.05.21.
//

#pragma once

#include <memory>

#include "mppi/core/config.h"
#include "mppi/core/policy.h"
#include "mppi/utils/savgol_filter.h"
#include "mppi/utils/multivariate_normal_eigen.h"

namespace mppi {

class  GaussianPolicy : public Policy {
public:
  GaussianPolicy(int nu, const Config& config);

  void update_samples(const std::vector<double>& weights, const int keep) override;

  void update(const std::vector<double>& weights, const double step_size) override;

  void shift(const double t) override;

  Eigen::VectorXd nominal(double t) override;

  Eigen::VectorXd sample(double t, int k) override;

  Eigen::VectorXd get_time() { return t_; }

  void bound() override;

 private:
  Config config_;
  double dt_;
  double h_;
  int ns_;
  int nt_;
  Eigen::ArrayXd t_;
  std::shared_ptr<multivariate_normal> dist_;
  std::vector<Eigen::MatrixXd> samples_;
  Eigen::MatrixXd nominal_;
  Eigen::MatrixXd delta_;
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> L_;  // matrix for shift operation of all the samples

  SavGolFilter filter_;
  Eigen::MatrixXd max_limits_;
  Eigen::MatrixXd min_limits_;

};
}