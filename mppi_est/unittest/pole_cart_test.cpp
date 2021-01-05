//
// Created by giuseppe on 05.01.21.
//

#include "pole_cart.h"
#include <gtest/gtest.h>
#include "mppi_est/model_particle_filter.hpp"

#include <fstream>
#include <iostream>

namespace mppi_est::pole_cart {
class ModelEstimatorTest : public ::testing::Test {
 protected:
  ModelEstimatorTest() {
    log_dir_ = __FILE__;
    log_dir_ = log_dir_.substr(0, log_dir_.rfind('/')) + "/logs/pole_cart";
    std::cout << "Log directory is: " << log_dir_ << std::endl;
    filter_ = std::make_unique<ModelParticleFilter>(1, true);
  }

  void SetUp() override {
    create_models();
    push_models();
  }

  void create_models() {
    pole_cart_params true_params;
    true_model_ = std::make_pair(std::make_unique<PoleCart>(true_params), true_params);

    std::ofstream params_file(log_dir_ + "/params.txt");

    size_t idx = 0;
    // TRUE value = 0.1
    for (double friction_cart = 0.0; friction_cart < 1.0; friction_cart += 0.1) {
      // TRUE value = 1.5
      for (double pole_length = 0.5; pole_length < 1.5; pole_length += 0.01) {
        pole_cart_params params;
        params.pole_length = pole_length;
        params.friction_cart = friction_cart;
        hypothesis_.push_back(std::make_pair(std::make_unique<PoleCart>(params), params));
        params_file << params;
        params_file << std::endl;
        idx++;
      }
    }
    params_file.close();
    std::cout << "Probing " << idx << " hypothesis." << std::endl;
  }

  void push_models() {
    double prior = 1.0 / hypothesis_.size();
    for (size_t i = 0; i < hypothesis_.size(); i++) filter_->add_model(prior, hypothesis_[i].first);
    std::cout << "There are: " << get_number_of_models() << " models." << std::endl;
  }

  size_t get_number_of_models() { return filter_->get_models().size(); }

  void run(size_t steps) {
    std::ofstream lh_log_file(log_dir_ + "/posterior.txt");
    std::ofstream sys_log_file(log_dir_ + "/system.txt");
    std::ofstream dev_log_file(log_dir_ + "/deviations.txt");

    double F = 2.0;  // Amplitude
    double T = 1.0;  // Period
    transition_tuple_t z;
    z.x.setZero(true_model_.first->get_measurement_dimension());
    z.x_next.setZero(true_model_.first->get_measurement_dimension());
    z.u.setZero(true_model_.first->get_action_dimension());

    double dt = 0.01;
    double t = 0;
    for (size_t i = 0; i < steps; i++) {
      z.u(0) = F * std::sin(i * dt * 2 * M_PI / T);
      true_model_.first->step(z);
      filter_->add_measurement_tuple(z);
      filter_->update_likelihood();
      filter_->update_posterior();

      sys_log_file << t << " " << z.x.transpose() << " " << z.u.transpose() << " "
                   << z.x_next.transpose() << std::endl;
      z.x = z.x_next;
      t += dt;

      for (size_t n = 0; n < get_number_of_models(); n++){
        lh_log_file << filter_->get_posterior()[n] << " ";
        dev_log_file << filter_->get_deviations()[n].norm() << " ";
      }
      lh_log_file << std::endl;
      dev_log_file << std::endl;
    }
    lh_log_file.close();
    sys_log_file.close();
    dev_log_file.close();
  }

 private:
  std::string log_dir_;
  std::unique_ptr<ModelParticleFilter> filter_;
  std::pair<std::unique_ptr<Model>, pole_cart_params> true_model_;
  std::vector<std::pair<std::unique_ptr<Model>, pole_cart_params>> hypothesis_;
};

TEST_F(ModelEstimatorTest, PoleCartTest) { run(1000); }
}  // namespace mppi_est::pole_cart

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
