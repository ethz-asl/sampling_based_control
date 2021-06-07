//
// Created by giuseppe on 31.05.21.
//

#include "mppi/policies/gaussian_policy.h"
#include "mppi/utils/utils.h"

using namespace mppi;

GaussianPolicy::GaussianPolicy(int nu, const Config& config)
    : Policy(nu) {
  std::cout << "Initializing policy from config: " << std::endl;
  std::cout << config;

  Eigen::MatrixXd C = config.input_variance.asDiagonal();
  dist_ = std::make_shared<multivariate_normal>(C);
  dt_ = config.step_size;
  ns_ = config.rollouts;

  nt_ = static_cast<int>(std::ceil(config.horizon / dt_));
  std::cout << "resizing samples" << std::endl;
  samples_.resize(ns_, Eigen::MatrixXd::Zero(nt_, nu));
  std::cout << "resized samples" << std::endl;

  t_ = Eigen::ArrayXd::LinSpaced(nt_, 0.0, nt_) * dt_;
  nominal_ = Eigen::MatrixXd::Zero(nt_, nu);
  delta_ = Eigen::MatrixXd::Zero(nt_, nu);
  std::cout << "here" << std::endl;

  L_.setIdentity(nt_);

  // Initialize filter
  filter_ = SavGolFilter(nt_, nu_, config.filters_window, config.filters_order);

  // initialize limits TODO(giuseppe) from params
  max_limits_ = Eigen::MatrixXd::Ones(nt_, nu_);
  min_limits_ = -Eigen::MatrixXd::Ones(nt_, nu_);

  for (int i=0; i < ns_; i++){
    max_limits_.row(i) = config.u_max;
    min_limits_.row(i) = config.u_min;
  }
}

Eigen::VectorXd GaussianPolicy::nominal(double t) {
  size_t time_idx = std::distance(
      t_.data(), std::upper_bound(t_.data(), t_.data() + t_.size(), t)) - 1;
  return nominal_.row(time_idx);
}

Eigen::VectorXd GaussianPolicy::sample(double t, int k) {
  size_t time_idx = std::distance(
      t_.data(), std::upper_bound(t_.data(), t_.data() + t_.size(), t)) - 1;
  return samples_[k].row(time_idx) + nominal_.row(time_idx);
}

void GaussianPolicy::update_samples(const std::vector<double>& weights,
                                    const int keep) {
  if (weights.size() != ns_){
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
    for (auto & sample : samples_)
    dist_->setRandomRow(sample);
  }
  else{
    std::vector<size_t> sorted_idxs = sort_indexes(weights);
    for (int i=keep; i<ns_-1; i++){
      dist_->setRandomRow(samples_[sorted_idxs[i]]);
    }
  }

  // noise free sample
  samples_[ns_-1].setZero();
  bound(); // TODO should bound each sample so that a convex combination is also within bounds
}

void GaussianPolicy::update(const std::vector<double>& weights,
                            const double step_size) {
  delta_ = nominal_;
  for (int i=0; i < ns_; i++){
    nominal_ += step_size * samples_[i] * weights[i];
  }

  filter_.reset(t_[0]);
  for (size_t i = 0; i < t_.size(); i++) {
    filter_.add_measurement(nominal_.row(i), t_(i));
  }
  for (size_t i = 0; i < t_.size(); i++) {
    filter_.apply(nominal_.row(i), t_(i));
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

    // TODO(giuseppe) investigate why this does not work
    // .rowwise() = nominal_.row(nt_ - time_idx_shift -1);
  }
}

void GaussianPolicy::bound() {
  for (auto& sample : samples_){
    sample = sample.cwiseMax(min_limits_).cwiseMin(max_limits_);
  }
}