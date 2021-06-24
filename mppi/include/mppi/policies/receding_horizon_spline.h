//
// Created by giuseppe on 03.06.21.
//

#pragma once

#include <iostream>
#include <algorithm>
#include <memory>
#include <Eigen/Core>
#include "mppi/utils/multivariate_normal_eigen.h"

namespace mppi { struct control_points; }

std::ostream &operator<<(std::ostream &os, const mppi::control_points &cp);

namespace mppi {

Eigen::ArrayXd B(const Eigen::ArrayXd &x, int k, int i,
                 const Eigen::ArrayXd &t);
Eigen::ArrayXd B2(const Eigen::ArrayXd &x, int k, int i,
                  const Eigen::ArrayXd &t);

struct BSplinePolicyConfig {
  double horizon = 1.0;
  int degree = 3;
  int samples = 10;
  double cp_dt = 0.1;
  double dt = 0.1;
  double sigma = 1.0;
  bool verbose = false;
  bool apply_bounds = false;
  double max_value;
  double min_value;
  double step_size;
};

struct control_points {
  control_points() = default;

  void resize(int n) {
    size_ = n;
    times_.resize(n);
    values_.resize(n);
  }

  size_t size() const { return size_; }

  void reset(double t);

  void shift_back(int i);

  size_t size_;
  Eigen::ArrayXd times_;
  Eigen::ArrayXd values_;
};

class RecedingHorizonSpline {
 public:
  RecedingHorizonSpline() = delete;
  RecedingHorizonSpline(const BSplinePolicyConfig &cfg);

  Eigen::ArrayXd get_time() { return t_; }

  Eigen::ArrayXd compute_nominal();

  void update_samples(const Eigen::VectorXd &weights, const int keep);

  Eigen::MatrixXd get_samples();

  Eigen::ArrayXd get_control_polygon_t() const { return c_points_.times_; }

  Eigen::ArrayXd get_control_polygon_y() const { return c_points_.values_; }

  Eigen::ArrayXd get_sample_control_polygon(const int i) {
    return N_.col(i) + c_points_.values_.matrix();
  }

  Eigen::ArrayXd get_variance();

  void update(const Eigen::VectorXd &weights, const double step_size);

  Eigen::MatrixXd get_gradients();

  Eigen::MatrixXd get_gradients_matrix() const;

  void shift(const double t);

  int get_time_idx(const double time);

 private:
  void gen_knots();

  void gen_control_points();

  Eigen::ArrayXd map_time_to_knots(const Eigen::ArrayXd &t);

  Eigen::ArrayXd map_knots_to_time(const Eigen::ArrayXd &k);

  Eigen::ArrayXd compute(const control_points &c, const Eigen::ArrayXd &time);

  Eigen::MatrixXd compute(const Eigen::MatrixXd &cm,
                          const Eigen::ArrayXd &time);

 public:
  void print_sample() { std::cout << P_ << std::endl; }

  Eigen::MatrixXd
      P_;  // matrix of the policy for each time and sample (nt x ns)
  Eigen::ArrayXd Pn_;  // vector of the nominal policy (nt)

  // TODO(giuseppe) make private again
 public:
  double sigma_max_;
  double sigma_;
  control_points c_points_;

 private:
  int n_;
  int m_;
  double t0_;
  double t_start_;
  double dt_;
  double cp_dt_;
  double horizon_;
  int n_steps_;
  int n_knots_;
  int n_cpoints_;
  int n_samples_;
  bool apply_bounds_;
  double max_value_;
  double min_value_;
  Eigen::MatrixXd max_value_matrix_;
  Eigen::MatrixXd min_value_matrix_;

  Eigen::ArrayXd knots_;

  // output times and values
  double time_shift_;
  Eigen::ArrayXd t_;
  Eigen::ArrayXd v_;

  // matrix of sample control points additive noise (nc x ns)
  Eigen::MatrixXd N_;

  // matrix of allocation coefficients for the b-splines (nt x nc)
  Eigen::MatrixXd M_;

  // matrix of inverse variance per control point (nc x nc)
  Eigen::MatrixXd Sinv_;

  // matrix of variance weights per time step (diagonal) (nt x nt)
  Eigen::MatrixXd V_;

  // matrix for shift operation of all the samples
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> L_;

  // matrix for ordering best samples
  Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic> O_;

  // sampler
  std::shared_ptr<multivariate_normal> dist_;
  double step_size_;
};
}