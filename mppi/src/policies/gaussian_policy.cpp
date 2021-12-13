//
// Created by giuseppe on 31.05.21.
//

#include "mppi/policies/gaussian_policy.h"
#include "mppi/utils/utils.h"

using namespace mppi;

GaussianPolicy::GaussianPolicy(int nu, const Config& config)
    : Policy(nu), config_(config) {
  Eigen::MatrixXd C = config.input_variance.asDiagonal();
  dist_ = std::make_shared<multivariate_normal>(C);
  dt_ = config.step_size;
  ns_ = config.rollouts;
  nt_ = static_cast<int>(std::ceil(config.horizon / dt_));
  samples_.resize(ns_, Eigen::MatrixXd::Zero(nt_, nu));

  // TODO(giuseppe) make this a parameter or find a better solution
  Eigen::VectorXd multipliers = Eigen::VectorXd::Zero(nt_);
  for (int i = 0; i < nt_; i++) {
    multipliers[i] = 1.0 - std::exp(-i / 20.0);
  }
  multipliers_ = multipliers.asDiagonal();

  t_ = Eigen::ArrayXd::LinSpaced(nt_, 0.0, nt_-1) * dt_;
  nominal_ = Eigen::MatrixXd::Zero(nt_, nu);
  nominal_temp_ = Eigen::MatrixXd::Zero(nu_, nu);
  delta_ = Eigen::MatrixXd::Zero(nt_, nu);
  gradient_ = Eigen::MatrixXd::Zero(nt_, nu);
  gradient2_ = Eigen::MatrixXd::Zero(nt_, nu);
  momentum_ = Eigen::MatrixXd::Zero(nt_, nu);
  momentum2_ = Eigen::MatrixXd::Zero(nt_, nu);
  momentum2_baseline_ = Eigen::MatrixXd::Ones(nt_, nu) * 1e-8;

  beta_1_ = config_.alpha;
  beta_2_ = config_.beta;
  if (config_.alpha < 1) adam_ = true;
//  std::cout << "Using adam? " << adam_ << std::endl;

  L_.setIdentity(nt_);

  // Initialize filter
  if (config_.filtering) {
    filter_ =
        SavGolFilter(nt_, nu_, config.filters_window, config.filters_order, 0, dt_);
  }

  // initialize limits
  max_limits_ = Eigen::MatrixXd::Ones(nt_, nu_);
  min_limits_ = -Eigen::MatrixXd::Ones(nt_, nu_);

  // used to clip the gradient
  max_gradient_ = Eigen::MatrixXd::Ones(nt_, nu_) * 0.2;

  for (int i = 0; i < nt_; i++) {
    max_limits_.row(i) = config.u_max;
    min_limits_.row(i) = config.u_min;
  }
  delay_steps_ = 0;
}

void GaussianPolicy::set_nominal(const Eigen::MatrixXd& nominal) {
  nominal_ = nominal;
}

void GaussianPolicy::update_delay(const int delay_steps) {
  Eigen::VectorXd multipliers = Eigen::VectorXd::Zero(nt_);
  for (int i = 0; i < nt_; i++) {
    multipliers(i) =
        (i < delay_steps) ? 0 : 1 - std::exp(-(i - delay_steps + 1) / 10.0);
  }
  multipliers_ = multipliers.asDiagonal();
  delay_steps_ = delay_steps;
}

Eigen::VectorXd GaussianPolicy::nominal(double t) {
  size_t time_idx =
      std::distance(t_.data(),
                    std::upper_bound(t_.data(), t_.data() + t_.size(), t)) -
      1;

  double alpha = (t - t_(time_idx)) / (t_(time_idx + 1) - t_(time_idx));
  return (1 - alpha) * nominal_.row(time_idx) +
         alpha * nominal_.row(time_idx + 1);
}

Eigen::VectorXd GaussianPolicy::sample(double t, int k) {
  size_t time_idx = std::distance(
      t_.data(), std::upper_bound(t_.data(), t_.data() + t_.size(), t)) - 1;
  return samples_[k].row(time_idx) + nominal_.row(time_idx);
}

void GaussianPolicy::update_samples(const std::vector<double>& weights,
                                    const int keep) {
  if ((int)weights.size() != ns_) {
    std::stringstream err;
    err << "Weights size does not match number of samples "
        << weights.size() << " != " << ns_ << std::endl;
    throw std::runtime_error(err.str());
  }

  if (keep > ns_){
    std::stringstream  err;
    err << "Trying to keep more samples than available. ";
    err << keep << " > " << ns_;
    throw std::runtime_error(err.str());
  }

  if (keep == 0){
    for (auto& sample : samples_) dist_->setRandomRow(sample);
  }
  else{
    std::vector<size_t> sorted_idxs = sort_indexes(weights);
    for (int i = keep; i < ns_ - 3; i++) {
      dist_->setRandomRow(samples_[sorted_idxs[i]]);
      samples_[sorted_idxs[i]] = samples_[sorted_idxs[i]];
    }

    // noise free sample
    samples_[sorted_idxs[ns_ - 2]].setZero();

    // sample exactly zero velocity
    samples_[sorted_idxs[ns_ - 1]] = -nominal_;
  }

  //  for (auto& sample : samples_) {
  //    sample = multipliers_ * sample;
  //  }

  bound(); // TODO should bound each sample so that a convex combination is also within bounds
}

void GaussianPolicy::update(const std::vector<double>& weights,
                            const double step_size) {
  delta_ = nominal_;
  nominal_temp_ = nominal_;
  gradient_.setZero();
  for (int i=0; i < ns_; i++){
    gradient_ += samples_[i] * weights[i];
  }

  // clip gradient
  gradient_ = gradient_.cwiseMax(-max_gradient_).cwiseMin(max_gradient_);

  
  if (adam_) {
    momentum_ = beta_1_ * momentum_ + (1.0 - beta_1_) * gradient_;
    gradient2_ = gradient_.cwiseProduct(gradient_);
    momentum2_ = beta_2_ * momentum2_ + (1.0 - beta_2_) * gradient2_;
    nominal_ +=
        0.01 *
        momentum_.cwiseProduct(
            (momentum2_.cwiseSqrt() + momentum2_baseline_).cwiseInverse());
  } else {
    nominal_ += step_size * gradient_;
  }

  // filter the temporary nominal 
  // if (config_.filtering) {
  //   filter_.reset(t_[0]);
  //   for (size_t i = 0; i < t_.size(); i++) {
  //     filter_.add_measurement(nominal_temp_.row(i), t_(i));
  //   }
  //   for (size_t i = 0; i < t_.size(); i++) {
  //     filter_.apply(nominal_temp_.row(i), t_(i));
  //   }
  // }

  // blend nominal trajectory with updates 
  //nominal_ = multipliers_ * (nominal_temp_ - nominal_) + nominal_;

  // update filter with new nominal
  if (config_.filtering) {
    filter_.reset(t_[0]);
    for (long int i = 0; i < t_.size(); i++) {
      filter_.add_measurement(nominal_.row(i), t_(i));
    }
    for (long int i = 0; i < t_.size(); i++) {
      filter_.apply(nominal_.row(i), t_(i));
    }
  }

  // update noise to current nominal trajectory
  delta_ = nominal_ - delta_;
  for (auto& sample : samples_)
    sample -= delta_;
}

void GaussianPolicy::shift(const double t) {
  static int time_idx_shift;
  if (t < t_[0]){
    throw std::runtime_error("Shifting back in time!");
  }

  if (t >= t_[1]){
    time_idx_shift = std::distance(
        t_.data(), std::upper_bound(t_.data(), t_.data() + t_.size(), t)) - 1;

    t_ += dt_ * time_idx_shift;

    if (time_idx_shift == nt_-1){
      Eigen::RowVectorXd last_nominal = nominal_.row(nt_-1);
      nominal_.rowwise() = last_nominal;
      for (auto& sample : samples_) sample.setZero();
      return;
    }

    L_.setIdentity();
    std::transform(L_.indices().data(), L_.indices().data() + nt_,
                   L_.indices().data(), [this](int i) -> int {
                     return (i < time_idx_shift) ? nt_ + i - time_idx_shift
                                                 : i - time_idx_shift;
                   });

    for (auto& sample : samples_){
      sample = L_ * sample;
      sample.bottomLeftCorner(time_idx_shift, nu_).setZero();
    }

    nominal_ = L_ * nominal_;
    nominal_.bottomLeftCorner(time_idx_shift, nu_).setZero();

    // extend the non-visited part with the last known value of momentum and
    // momentum2
    if (adam_) {
      momentum_ = L_ * momentum_;
      momentum_.bottomLeftCorner(time_idx_shift, nu_).rowwise() =
          momentum_.row(nt_ - time_idx_shift - 1);  //.setZero();
      momentum2_ = L_ * momentum2_;
      momentum2_.bottomLeftCorner(time_idx_shift, nu_).rowwise() =
          momentum2_.row(nt_ - time_idx_shift - 1);
    }

    // TODO(giuseppe) investigate why this does not work
    // .rowwise() = nominal_.row(nt_ - time_idx_shift -1);
  }
}

void GaussianPolicy::bound() {
  for (auto& sample : samples_){
    sample = sample.cwiseMax(min_limits_).cwiseMin(max_limits_);
  }
}