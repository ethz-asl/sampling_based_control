//
// Created by giuseppe on 12/16/20.
//

#pragma once
#include <Eigen/Core>
#include <boost/circular_buffer.hpp>
#include <vector>

#include "mppi_est/model.hpp"
#include "mppi_est/typedefs.hpp"

using namespace Eigen;

namespace mppi_est {

class ModelParticleFilter {
 public:
  using model_ptr_t = std::unique_ptr<Model>;

  explicit ModelParticleFilter(size_t buffer_length, bool debug = false);
  ModelParticleFilter() : ModelParticleFilter(1){};
  ~ModelParticleFilter() = default;

  void add_model(const double prior, const model_ptr_t& model);
  void add_measurement_tuple(const transition_tuple_t& z);
  void update_likelihood();
  void update_posterior();

  inline const std::vector<model_ptr_t>& get_models() const { return models_; }
  std::vector<double> get_posterior() const { return posterior_; }
  std::vector<vector_t> get_deviations() const { return deviations_; }

 private:
  void init_from_model(const model_ptr_t& model);
  [[nodiscard]] bool check_model(const model_ptr_t& model) const;

 private:
  bool debug_;
  bool initialized_;
  size_t samples_;
  size_t buffer_length_;

  std::vector<double> posterior_;
  std::vector<double> likelihood_;
  std::vector<vector_t> deviations_;
  matrix_t sigma_;
  matrix_t sigma_inv_;

  size_t no_;
  size_t nu_;
  std::vector<std::unique_ptr<Model>> models_;
  boost::circular_buffer<transition_tuple_t> z_buffer_;
};

}  // namespace mppi_est
