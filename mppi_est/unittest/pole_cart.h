//
// Created by giuseppe on 05.01.21.
//

#pragma once
#include "mppi_est/model.hpp"

namespace mppi_est::pole_cart {

struct pole_cart_params {
  double mass_cart = 1.0;
  double mass_pole = 1.0;
  double pole_length = 1.0;
  double friction_cart = 0.1;
  double friction_pole = 0.1;
  // cov_x, cov_theta, cov_xd, cov_thetad
  std::array<double, 4> cov{0.01, 0.01, 0.01, 0.01};
};

class PoleCart : public Model {
 public:
  explicit PoleCart(const pole_cart_params& params);
  std::unique_ptr<Model> create() override { return std::make_unique<PoleCart>(params_); }
  void reset(const vector_t& x) override;
  void step(const transition_tuple_t& z) override;
  void display_model_params() const override;

 private:
  double dt_ = 0.01;
  pole_cart_params params_;
};
}  // namespace mppi_est::pole_cart

void operator<<(std::ostream& os, const mppi_est::pole_cart::pole_cart_params& p);
