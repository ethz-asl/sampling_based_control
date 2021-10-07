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

void control_points::reset(double t) { times_ = times_ - times_[0] + t; }

void control_points::shift_back(int i) {
  double last_time = times_(size_ - 1);
  double dt = times_(size_ - 1) - times_(size_ - 2);  // infer time delta

  std::rotate(times_.data(), times_.data() + i, times_.data() + size_);
  std::rotate(values_.data(), values_.data() + i, values_.data() + size_);

  // TODO(giuseppe) filling with zeros is based on the heuristics that 0 is a
  // stable policy -> this should be problem dependent std::fill(values_.data()
  // + size_ - i, values_.data() + size_, last_value);
  std::fill(values_.data() + size_ - i, values_.data() + size_, 0.0);

  for (int k = i; k > 0; k--) {
    last_time += dt;
    times_(size_ - k) = last_time;
  }
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

  // bounds
  apply_bounds_ = cfg.apply_bounds;
  max_value_ = cfg.max_value;
  min_value_ = cfg.min_value;
  max_value_matrix_ =
      Eigen::MatrixXd::Ones(n_cpoints_, n_samples_) * max_value_;
  min_value_matrix_ =
      Eigen::MatrixXd::Ones(n_cpoints_, n_samples_) * min_value_;

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
  sigma_ = cfg.sigma;
  sigma_max_ = cfg.sigma;
  Eigen::MatrixXd C = cfg.sigma * Eigen::VectorXd::Ones(n_cpoints_).asDiagonal();
  dist_ = std::make_shared<multivariate_normal>(C);
  step_size_ = cfg.step_size;

  // allocate memory for sampling
  N_ = Eigen::MatrixXd::Zero(n_cpoints_, n_samples_);
  M_ = Eigen::MatrixXd::Zero(t_.size(), n_cpoints_);
  V_ = Eigen::MatrixXd::Zero(t_.size(), t_.size());
  P_ = Eigen::MatrixXd::Zero(t_.size(), n_samples_);
  Pn_ = Eigen::VectorXd::Zero(t_.size());
  Sinv_ = Eigen::VectorXd::Ones(n_cpoints_).asDiagonal() * (1.0 / cfg.sigma);
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
  c_points_.values_.setZero();
  c_points_.values_ =
      c_points_.values_.cwiseMax(min_value_).cwiseMin(max_value_);
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

  static int n_cpoints_shift;
  if (t > t_start_ + horizon_) {
    std::cout << t
              << " beyond horizon: shifting all control points. (t start = "
              << t_start_ << ")" << std::endl;
    c_points_.reset(t);
    std::cout << "control points reset to t = " << t << std::endl;
    t_start_ = t;

    // throw std::runtime_error("Trying to shift further than the horizon");
  } else {
    n_cpoints_shift = static_cast<int>(std::floor((t - t_start_) / cp_dt_));
    if (n_cpoints_shift > 0) {
      c_points_.shift_back(n_cpoints_shift);
      t_start_ = t;  // c_points_.times_[0];

      // permute noise matrix
      L_.setIdentity();
      std::transform(L_.indices().data(), L_.indices().data() + n_cpoints_,
                     L_.indices().data(), [this](int i) -> int {
                       return (i < n_cpoints_shift)
                                  ? this->n_cpoints_ + i - n_cpoints_shift
                                  : i - n_cpoints_shift;
                     });
      N_ = L_ * N_;

      // set the last samples to 0
      // start row, start col, block rows, block cols
      N_.block(n_cpoints_ - n_cpoints_shift, 0, n_cpoints_shift, n_samples_)
          .setZero();
    }
  }

  t0_ = t;
  t_ = t_ + (t0_ - t_(0));  // shift also all fine control times

  // update nominal policy and samples
  Pn_ = compute_nominal();
  P_ = compute(N_, t_).colwise() + Pn_.matrix();
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

  // Set one sample completely to zero (when summed with control points)
  N_.col(n_samples_ - 2) = -c_points_.values_;

  // Freeze cpoints that control the first interval
  //N_.topRows(m_).setZero();

  // Compute new samples (including nominal trajectory as last sample)
  // Compute capped control points
  if (apply_bounds_) {
    N_.colwise() += c_points_.values_.matrix();
    N_ = N_.cwiseMax(min_value_matrix_).cwiseMin(max_value_matrix_);
    N_.colwise() -= c_points_.values_.matrix();
  }

  P_ = compute(N_, t_).colwise() + compute_nominal().matrix();
}

Eigen::MatrixXd RecedingHorizonSpline::get_gradients() {
  V_ = get_variance().cwiseInverse().matrix().asDiagonal();
  return compute(N_, t_).transpose() * V_ * M_ * Sinv_;
}

Eigen::MatrixXd RecedingHorizonSpline::get_gradients_matrix() const {
  return V_ * M_ * Sinv_;
}

void RecedingHorizonSpline::update(const Eigen::VectorXd &weights,
                                   const double step_size) {
  Eigen::VectorXd c_points_temp = c_points_.values_;
  Eigen::VectorXd delta_temp = /*step_size * */ step_size_ * sigma_ *
                               (weights.transpose() * get_gradients()).array();
  //std::cout << "delta temp is: " << delta_temp.transpose() << std::endl;

  // TODO(giuseppe) for now controlling gradients using normalization
  // delta_temp.normalize();

  // update variance
  bool update_variance = false;

  if (update_variance) {
    V_ = get_variance().cwiseInverse().matrix().asDiagonal();
    // std::cout << "variance matrix is: \n" << V_ << std::endl;
    // Eigen::MatrixXd noise_matrix = compute(N_, t_);
    // std::cout << "noise matrix is: \n" << noise_matrix << std::endl;

    Eigen::MatrixXd Eps2 = compute(N_, t_).array().pow(2).matrix();
    // std::cout << "noise squared matrix is: \n" << Eps2 << std::endl;
    // std::cout << "weights are: \n" << weights.transpose() << std::endl;
    // std::cout << "weighted noise matrix is: \n" << V_ * Eps2 * weights <<
    // std::endl; std::cout << "summing up: \n" <<
    // Eigen::VectorXd::Ones(t_.size()).transpose() * V_ * Eps2 * weights <<
    // std::endl;

    auto weighted_sigma = 1. / t_.size() *
                          Eigen::VectorXd::Ones(t_.size()).transpose() * V_ *
                          Eps2 * weights;
    auto empiric_sigma =
        1. / t_.size() * Eigen::VectorXd::Ones(t_.size()).transpose() * V_ *
        Eps2 * Eigen::VectorXd::Ones(t_.size()) * 1.0 / n_samples_;

    std::cout << "weighted sigma: " << weighted_sigma << std::endl;
    std::cout << "empiric sigma: " << empiric_sigma << std::endl;
    std::cout << "delta sigma: " << sigma_ - weighted_sigma[0] << std::endl;
    double sigma_inv = 1.0 / sigma_;
    sigma_inv = sigma_inv + sigma_inv * (sigma_ - weighted_sigma[0]);
    sigma_inv = std::min(std::max(sigma_inv, 1. / sigma_max_),
                         100.0);  // corresponindg to min sigma = 0.001
    sigma_ = 1.0 / sigma_inv;

    // dist_->set_covariance(sigma_ *
    // Eigen::VectorXd::Ones(n_cpoints_).asDiagonal());
    std::cout << "New sigma is: " << 1. / sigma_inv << std::endl;
  }

  //  std::cout << "=====================================" << std::endl;
  //  std::cout << "c_points temp: " << c_points_temp.transpose() << std::endl;
  //  std::cout << "delta temp: " << delta_temp.transpose() << std::endl;

  // TODO(giuseppe) attempt to enforce smoothness
  // delta_temp.head(m_+1).setZero();

  c_points_temp += delta_temp;
  //  std::cout << "c_points temp + delta: " << c_points_temp.transpose() <<
  //  std::endl;

  // make sure that control points stay within bounds
  c_points_.values_ = c_points_temp.cwiseMin(max_value_).cwiseMax(min_value_);
  //  std::cout << "c_points clamped: " << c_points_.values_.transpose() <<
  //  std::endl;

  Eigen::VectorXd delta =
      delta_temp - (c_points_temp - c_points_.values_.matrix());
  //  std::cout << "max delta: " << delta.transpose() << std::endl;

  // update noise (deviation) wrt to new control polygon
  N_.colwise() -= delta;

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
