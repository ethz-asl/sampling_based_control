//
// Created by giuseppe on 12/16/20.
//

#include "mppi_est/model.hpp"

namespace mppi_est {

Model::Model(size_t state_dim, size_t action_dim) : no_(state_dim), nu_(action_dim) {
  cov_.setConstant(no_, no_, 1.0);
  cov_inv_.setConstant(no_, no_, 1.0);
}

Model::Model(size_t state_dim, size_t action_dim, const vector_t& cov)
    : Model(state_dim, action_dim) {
  set_measurement_covariance(cov);
}

void Model::set_measurement_covariance(const vector_t& cov) {
  if (cov.size() != no_) {
    std::stringstream ss;
    ss << "Covariance vector has wrong size: " << cov.size() << "!=" << no_;
    PRINT_ERROR(ss.str());
    return;
  }
  cov_ = cov.asDiagonal();
  cov_inv_.setZero(no_, no_);
  for (size_t i = 0; i < no_; i++) {
    cov_inv_(i, i) = cov(i) < 1e-6 ? 1e6 : 1. / cov(i);
  }
}
}  // namespace mppi_est