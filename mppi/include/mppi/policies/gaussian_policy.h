//
// Created by giuseppe on 31.05.21.
//

#pragma once

#include <memory>

#include "mppi/core/config.h"
#include "mppi/core/policy.h"
#include "mppi/utils/savgol_filter.h"
#include "mppi/utils/multivariate_normal_eigen.h"

/// Implementation of a gaussian policy that perturbs the nominal
/// trajectory with gaussian noise and applies a Savitzky-Golay filter
/// to the resulting nominal input sequence

namespace mppi {

class  GaussianPolicy : public Policy {
public:
  GaussianPolicy(int nu, const Config& config);

  void update_samples(const std::vector<double>& weights, const int keep) override;

  void update(const std::vector<double>& weights, const double step_size) override;

  void update_delay(const int delay_steps) override;

  void shift(const double t) override;

  Eigen::VectorXd nominal(double t) override;

  Eigen::VectorXd sample(double t, int k) override;

  Eigen::VectorXd get_time() { return t_; }

  void bound() override;

  void set_nominal(const Eigen::MatrixXd& nominal) override;

 private:
  Config config_;
  double dt_;
  double h_;
  int ns_;
  int nt_;
  Eigen::ArrayXd t_;
  std::shared_ptr<multivariate_normal> dist_;
  Eigen::MatrixXd multipliers_;  // scale samples for each time step
  std::vector<Eigen::MatrixXd> samples_;
  Eigen::MatrixXd nominal_;
  Eigen::MatrixXd nominal_temp_;
  Eigen::MatrixXd delta_;
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> L_;  // matrix for shift operation of all the samples

  SavGolFilter filter_;
  Eigen::MatrixXd max_limits_;
  Eigen::MatrixXd min_limits_;

  bool adam_ = false;
  double beta_1_ = 0.9;
  double beta_2_ = 0.999;
  Eigen::MatrixXd gradient_;
  Eigen::MatrixXd gradient2_;
  Eigen::MatrixXd max_gradient_;  
  Eigen::MatrixXd momentum_;
  Eigen::MatrixXd momentum2_;
  Eigen::MatrixXd momentum2_baseline_;

  int delay_steps_;
};
}