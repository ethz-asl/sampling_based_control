//
// Created by giuseppe on 12/16/20.
//

#include <numeric>
#include "mppi_est/model_particle_filter.hpp"

namespace mppi_est {

ModelParticleFilter::ModelParticleFilter(const size_t buffer_length)
    : buffer_length_(buffer_length) {
  samples_ = 0;
  z_buffer_.rset_capacity(buffer_length_);
}

void ModelParticleFilter::add_model(const double prior,
                                    const model_ptr_t& model) {
  models_.push_back(model->create());
  prior_.push_back(prior);
  likelihood_.push_back(0.0);
  samples_++;
}

void ModelParticleFilter::update_likelihood() {
  transition_tuple_t z_r = z_buffer_.back();
  transition_tuple_t z_s = z_r;
  for (size_t i = 0; i < samples_; i++) {
    models_[i]->step(z_s);
    likelihood_[i] = likelihood_fn(z_r, z_s);
    prior_[i] = prior_[i] * likelihood_[i];
  }
}

void ModelParticleFilter::update_posterior() {
  double eta = std::accumulate(prior_.begin(), prior_.end(), 0.0);
  for (size_t i=0; i < samples_; i++){
    prior_[i] = prior_[i] / eta;
  }
}

void ModelParticleFilter::add_measurement_tuple(const transition_tuple_t& z) {
  z_buffer_.push_back(z);
}

double ModelParticleFilter::likelihood_fn(const transition_tuple_t& z1,
                                          const transition_tuple_t& z2) {
  return (z1.x_next - z2.x_next).transpose() * sigma_inv_ *
         (z1.x_next - z2.x_next)
}
}  // namespace mppi_est