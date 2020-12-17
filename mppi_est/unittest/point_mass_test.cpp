//
// Created by giuseppe on 12/17/20.
//

#include "point_mass.h"
#include <gtest/gtest.h>
#include "mppi_est/model_particle_filter.hpp"

#include <fstream>
#include <iostream>

namespace mppi_est::test {
class ModelEstimatorTest : public ::testing::Test {
 protected:
  ModelEstimatorTest() {
    log_dir_ = __FILE__;
    log_dir_ = log_dir_.substr(0, log_dir_.rfind('/')) + "/logs/";
    std::cout << "Log directory is: " << log_dir_ << std::endl;
  }

  void SetUp() override {
    create_models();
    push_models();
  }

  void create_models() {
    point_mass_params true_params;
    true_params.mu = 1.0;
    true_model_ = std::make_pair(std::make_unique<PointMass>(true_params), true_params);

    std::ofstream params_file(log_dir_ + "/params.txt");
    double spacing = (10.0 - 0.0) / 20.0;
    for (size_t i = 0; i < 20; i++) {
      point_mass_params params;
      params.mu = i * spacing;
      hypothesis_[i] = std::make_pair(std::make_unique<PointMass>(params), params);
      params_file << params.mu << " ";
    }
    params_file << std::endl;
    params_file.close();
  }

  void push_models() {
    double prior = 1 / 20.0;
    for (size_t i = 0; i < 20; i++) filter_.add_model(prior, hypothesis_[i].first);
  }

  size_t get_number_of_models() { return filter_.get_models().size(); }

  void run(size_t steps) {
    std::ofstream lh_log_file(log_dir_ + "/posterior.txt");
    std::ofstream sys_log_file(log_dir_ + "/system.txt");

    double F = 2.0;
    double T = 1.0;
    transition_tuple_t z;
    z.x.setZero(2);
    z.x_next.setZero(2);
    z.u.setZero(1);

    double dt = 0.01;
    double t = 0;
    for (size_t i = 0; i < steps; i++) {
      z.u(0) = F * std::sin(i*dt * 2 * M_PI / T);
      true_model_.first->step(z);
      filter_.add_measurement_tuple(z);
      filter_.update_likelihood();
      filter_.update_posterior();

      sys_log_file << t << " " << z.x.transpose() << " " << z.u.transpose() << " " << z.x_next.transpose()
                   << std::endl;
      z.x = z.x_next;
      t += dt;

      for (size_t n = 0; n < get_number_of_models(); n++)
        lh_log_file << filter_.get_posterior()[n] << " ";
      lh_log_file << std::endl;
    }
    lh_log_file.close();
    sys_log_file.close();
  }

 private:
  std::string log_dir_;
  ModelParticleFilter filter_;
  std::pair<std::unique_ptr<Model>, point_mass_params> true_model_;
  std::array<std::pair<std::unique_ptr<Model>, point_mass_params>, 20> hypothesis_;
};

TEST_F(ModelEstimatorTest, FilterTest) {
  ASSERT_EQ(get_number_of_models(), 20);
  run(1000);
}

}  // namespace mppi_est::test

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
