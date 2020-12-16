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

  ModelParticleFilter(size_t buffer_length);
  ModelParticleFilter() : ModelParticleFilter(1){};
  ~ModelParticleFilter() = default;

  void add_model(const double prior, const model_ptr_t& model);
  void add_measurement_tuple(const transition_tuple_t& z);
  void update_likelihood();
  void update_posterior();
  void get_posterior();
  void sample();

 private:
  double likelihood_fn(const transition_tuple_t& z1,
                       const transition_tuple_t& z2);

 private:
  size_t samples_;
  size_t buffer_length_;

  std::vector<double> prior_;
  std::vector<double> likelihood_;
  matrix_t sigma_;
  matrix_t sigma_inv_;

  std::vector<std::unique_ptr<Model>> models_;
  boost::circular_buffer<transition_tuple_t> z_buffer_;
};

}  // namespace mppi_est
