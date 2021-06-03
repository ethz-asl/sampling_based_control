//
// Created by giuseppe on 03.06.21.
//

#include "mppi/policies/receding_horizon_spline.h"
#include "mppi/utils/utils.h"

#include <cassert>

using namespace mppi;

Eigen::ArrayXd mppi::B(const Eigen::ArrayXd &x, int k, int i,
                         const Eigen::ArrayXd &t) {
  assert(t.size() >= k + 1 &&
         "Base spline of order k requires at least support of size k+1");

  if (k == 0) return ((x >= t(i)) && (x < t(i + 1))).cast<double>();

  Eigen::ArrayXd c1(x.size()), c2(x.size());
  if (t(i + k) == t(i))
    c1.setZero();
  else
    c1 = (1 / (t(i + k) - t(i))) *
         (x - t(i)).cwiseProduct(mppi::B(x, k - 1, i, t));

  if (t(i + k + 1) == t(i + 1))
    c2.setZero();
  else
    c2 = (1 / (t(i + k + 1) - t(i + 1))) *
         (t(i + k + 1) - x).cwiseProduct(mppi::B(x, k - 1, i + 1, t));
  return c1 + c2;
}

Eigen::ArrayXd mppi::B2(const Eigen::ArrayXd &x, int k, int i,
                          const Eigen::ArrayXd &t) {
  assert(t.size() >= k + 1 &&
         "Base spline of order k requires at least support of size k+1");

  if (k == 0) return ((x >= t(i)) && (x < t(i + 1))).cast<double>();

  Eigen::ArrayXd c1(x.size()), c2(x.size());
  if (t(i + k) == t(i))
    c1.setZero();
  else
    c1 = (1 / (t(i + k) - t(i))) *
         (x - t(i)).cwiseProduct(mppi::B(x, k - 1, i, t));

  if (t(i + k + 1) == t(i + 1))
    c2.setZero();
  else
    c2 = (1 / (t(i + k + 1) - t(i + 1))) *
         (t(i + k + 1) - x).cwiseProduct(mppi::B(x, k - 1, i + 1, t));
  return c1 * c1 + c2 * c2 + 2 * c1 * c2;
}

RecedingHorizonSpline::RecedingHorizonSpline(const BSplinePolicyConfig &cfg) {
  n_ = cfg.degree;
  m_ = n_ + 1;
  horizon_ = cfg.horizon;
  n_steps_ = static_cast<int>(std::floor(horizon_ / cfg.cp_dt));
  n_knots_ = n_steps_ + n_ + m_;
  n_samples_ = cfg.samples;

  // need additional knot interval to use when querying horizon with t0 between
  // first and last knot
  n_knots_ = n_knots_ + 1;
  n_cpoints_ = n_knots_ - m_;
  cp_dt_ = horizon_ / n_steps_;

  t_start_ = 0.0;
  t0_ = 0.0;
  time_shift_ = -0.5 * cp_dt_ * m_;

  // adjust discretization
  int substeps = static_cast<int>(std::floor(horizon_ / cfg.dt)) + 1;
  t_.resize(substeps);
  v_.resize(substeps);
  for (int i = 0; i < substeps; i++) {
    t_(i) = t_start_ + i * cfg.dt;
  }

  // init data
  gen_knots();
  gen_control_points();

  // normal distribution
  Eigen::MatrixXd C = cfg.sigma * Eigen::VectorXd::Ones(n_cpoints_).asDiagonal();
  dist_ = std::make_shared<multivariate_normal>(C);

  // allocate memory for sampling
  N_ = Eigen::MatrixXd::Zero(n_cpoints_, n_samples_);
  M_ = Eigen::MatrixXd::Zero(t_.size(), n_cpoints_);
  V_ = Eigen::MatrixXd::Zero(t_.size(), t_.size());
  P_ = Eigen::MatrixXd::Zero(t_.size(), n_samples_);
  Pn_ = Eigen::VectorXd::Zero(t_.size());
  S_ = Eigen::VectorXd::Ones(n_cpoints_).asDiagonal() * cfg.sigma;
  L_.setIdentity(n_cpoints_);
  O_.setIdentity(n_samples_);
  dist_->setRandom(N_);

  if (cfg.verbose) {
    std::cout << "num control points: " << n_cpoints_ << std::endl;
    std::cout << c_points_ << std::endl;
    std::cout << "knots are: " << knots_.transpose() << std::endl;
  }
}

void RecedingHorizonSpline::gen_knots() {
  knots_ = Eigen::ArrayXd::Zero(n_knots_);
  for (int i = 0; i < n_knots_; i++) knots_(i) = i - n_;
}

void RecedingHorizonSpline::gen_control_points() {
  c_points_.resize(n_cpoints_);
  c_points_.times_ = knots_.head(n_cpoints_) * cp_dt_ - time_shift_;
  c_points_.values_.setRandom();
}

Eigen::ArrayXd RecedingHorizonSpline::compute(const control_points &c,
                                              const Eigen::ArrayXd &t) {
  Eigen::ArrayXd x = map_time_to_knots(t);
  Eigen::ArrayXd v = Eigen::ArrayXd::Zero(x.size());
  for (int i = 0; i < c.size(); i++) v += c.values_(i) * B(x, n_, i, knots_);
  return v;
}

Eigen::MatrixXd RecedingHorizonSpline::compute(const Eigen::MatrixXd &cm,
                                               const Eigen::ArrayXd &t) {
  Eigen::ArrayXd x = map_time_to_knots(t);
  for (int i = 0; i < cm.rows(); i++) M_.col(i) = B(x, n_, i, knots_);
  Eigen::MatrixXd res;
  res = M_ * cm;
  return res;
}

Eigen::ArrayXd RecedingHorizonSpline::get_variance() {
  Eigen::ArrayXd x = map_time_to_knots(t_);
  Eigen::ArrayXd v = Eigen::ArrayXd::Zero(x.size());
  for (int i = 0; i < c_points_.size(); i++) v += B2(x, n_, i, knots_);
  return v;
}

void RecedingHorizonSpline::shift(const double t) {
  if (t < t0_) throw std::runtime_error("Trying to shift back in time!");
  if (t > t_start_ + horizon_)
    throw std::runtime_error("Trying to shift further than the horizon");

  static int n_cpoints_shift;
  n_cpoints_shift = static_cast<int>(std::floor((t - t_start_) / cp_dt_));
  if (n_cpoints_shift > 0) {
    c_points_.shift_back(n_cpoints_shift);
    t_start_ = t;

    // permute noise matrix
    L_.setIdentity();
    std::transform(L_.indices().data(), L_.indices().data() + n_cpoints_,
                   L_.indices().data(), [this](int i) -> int {
          return (i<n_cpoints_shift) ? this->n_cpoints_ - n_cpoints_shift: i-n_cpoints_shift;
        });
    N_ = L_ * N_;

    // set the last samples to 0
    // start row, start col, block rows, block cols
    N_.block(n_cpoints_ - n_cpoints_shift, 0, n_cpoints_shift, n_samples_).setZero();
  }
  t0_ = t;
  t_ = t_ + (t0_ - t_(0));  // shift also all fine control times
}

Eigen::ArrayXd RecedingHorizonSpline::map_time_to_knots(const Eigen::ArrayXd &t) {
  return knots_(n_) + (t - t_start_) / cp_dt_;
}

Eigen::ArrayXd RecedingHorizonSpline::map_knots_to_time(const Eigen::ArrayXd &k){}

Eigen::ArrayXd RecedingHorizonSpline::compute_nominal() {
  return compute(c_points_, t_);
}

Eigen::MatrixXd RecedingHorizonSpline::get_samples() {
  return P_;
}

void RecedingHorizonSpline::update_samples(const Eigen::VectorXd& weights, const int keep) {
  if (weights.size() != n_samples_){
    std::stringstream err;
    err << "Weights size does not match number of samples "
        << weights.size() << " != " << n_samples_ << std::endl;
    throw std::runtime_error(err.str());
  }

  if (keep > n_samples_){
    std::stringstream  err;
    err << "Trying to keep more samples than available. ";
    err << keep << " > " << n_samples_;
    throw std::runtime_error(err.str());
  }

  if (keep == 0){
    dist_->setRandom(N_);
  }
  else{
    std::vector<size_t> sorted_idxs = sort_indexes(weights);
    O_.setIdentity();
    std::transform(O_.indices().data(), O_.indices().data() + n_samples_, O_.indices().data(),
                   [&](int i) { return sorted_idxs[i]; });
    N_ = N_ * O_;
    dist_->setRandom(N_.bottomRightCorner(n_cpoints_, n_samples_ - keep));
  }

  // Keep one sample noise-free
  N_.col(n_samples_-1).setZero();

  // Compute new samples (including nominal traj as last sample)
  P_ = compute(N_, t_).colwise() + compute_nominal().matrix();
}

Eigen::MatrixXd RecedingHorizonSpline::get_gradients() {
  V_ = get_variance().matrix().asDiagonal();
  return compute(N_, t_).transpose() * V_ * M_ * S_;
}

Eigen::MatrixXd RecedingHorizonSpline::get_gradients_matrix() const {
  return V_ * M_ * S_;
}

void RecedingHorizonSpline::update(const Eigen::VectorXd &weights,
                                   const double step_size) {
  Eigen::ArrayXd delta =
      step_size * (weights.transpose() * get_gradients()).array();
  c_points_.values_ += delta;
  N_.colwise() -= delta.matrix();  // the noise wrt to the current newly defined
  // control polygon
  Pn_ = compute_nominal();
}

int RecedingHorizonSpline::get_time_idx(const double time) {
  return std::distance(t_.data(), std::upper_bound(t_.data(), t_.data() + t_.size(), time)) - 1;
}

std::ostream &operator<<(std::ostream &os, const control_points &cp) {
  os << "Control points: " << std::endl;
  os << "- values: " << cp.values_.transpose() << std::endl;
  os << "- times: " << cp.times_.transpose() << std::endl;
  return os;
}
