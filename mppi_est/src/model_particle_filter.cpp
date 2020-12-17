//
// Created by giuseppe on 12/16/20.
//

#include "mppi_est/model_particle_filter.hpp"
#include <iostream>
#include <numeric>

namespace mppi_est {

ModelParticleFilter::ModelParticleFilter(const size_t buffer_length)
    : initialized_(false), buffer_length_(buffer_length) {
  samples_ = 0;
  z_buffer_.rset_capacity(buffer_length_);
}

void ModelParticleFilter::init_from_model(const model_ptr_t& model) {
  no_ = model->get_measurement_dimension();
  nu_ = model->get_action_dimension();
}

bool ModelParticleFilter::check_model(const model_ptr_t& model) const {
  bool success = true;
  std::stringstream error;
  if (model->get_measurement_dimension() != no_) {
    error << "Model has the wrong observation size: " << model->get_measurement_dimension()
          << " != " << no_ << std::endl;
    PRINT_ERROR(error.str());
    success = false;
  }
  if (model->get_action_dimension() != nu_) {
    error << "Model has the wrong action size: " << model->get_action_dimension() << " != " << nu_
          << std::endl;
    PRINT_ERROR(error.str());
    success = false;
  }
  return success;
}

void ModelParticleFilter::add_model(const double prior, const model_ptr_t& model) {
  models_.push_back(model->create());
  posterior_.push_back(prior);
  likelihood_.push_back(0.0);
  bool success = true;
  if (samples_ == 0)
    init_from_model(model);
  else
    success = check_model(model);

  if (!success)
    PRINT_ERROR(std::string("Failed to add the model"));
  else
    samples_++;
}

void ModelParticleFilter::update_likelihood() {
  transition_tuple_t zr = z_buffer_.back();
  transition_tuple_t zs = zr;
  for (size_t i = 0; i < samples_; i++) {
    models_[i]->reset(zr.x);
    models_[i]->step(zs);
    vector_t delta = zs.x_next - zr.x_next;
    likelihood_[i] = std::exp(-delta.transpose() * models_[i]->get_covariance_inv() * delta);
    posterior_[i] = posterior_[i] * likelihood_[i];
  }
}

void ModelParticleFilter::update_posterior() {
  double eta = std::accumulate(posterior_.begin(), posterior_.end(), 0.0);
  for (size_t i = 0; i < samples_; i++) {
    posterior_[i] = posterior_[i] / eta;
  }
}

void ModelParticleFilter::add_measurement_tuple(const transition_tuple_t& z) {
  z_buffer_.push_back(z);
}

}  // namespace mppi_est