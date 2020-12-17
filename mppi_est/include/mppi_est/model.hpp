//
// Created by giuseppe on 12/16/20.
//

#pragma once

#include <memory>
#include "mppi_est/typedefs.hpp"

namespace mppi_est {

class Model {
 public:
  Model() = delete;
  Model(size_t state_dim, size_t action_dim);
  Model(size_t state_dim, size_t action_dim, const vector_t& cov);
  ~Model() = default;

 public:
  [[nodiscard]] inline size_t get_measurement_dimension() const { return no_; };
  [[nodiscard]] inline size_t get_action_dimension() const { return nu_; }

  virtual std::unique_ptr<Model> create() = 0;
  virtual void reset(const vector_t& x) = 0;
  virtual void step(const transition_tuple_t& z) = 0;

  void set_measurement_covariance(const vector_t& cov);
  [[nodiscard]] inline const matrix_t& get_covariance() const { return cov_; }
  [[nodiscard]] inline const matrix_t& get_covariance_inv() const { return cov_inv_; }

 private:
  size_t no_;
  size_t nu_;
  matrix_t cov_;
  matrix_t cov_inv_;
};
}  // namespace mppi_est