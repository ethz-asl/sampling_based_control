//
// Created by giuseppe on 05.01.21.
//

#include "pole_cart.h"

namespace mppi_est::pole_cart {

PoleCart::PoleCart(const pole_cart_params& params) : Model(4, 1), params_(params) {
  vector_t cov;
  cov.resize(4);
  cov << params_.cov[0], params_.cov[1], params_.cov[2], params_.cov[3];
  set_measurement_covariance(cov);
}

void PoleCart::reset(const vector_t& x) { assert(x.size() == get_measurement_dimension()); }

void PoleCart::step(const transition_tuple_t& z) {
  assert(z.u.size() == get_action_dimension());
  assert(z.x.size() == get_measurement_dimension());
  assert(z.x_next.size() == get_measurement_dimension());

  // x(0) : cart position
  // x(1) : pole position
  // x(2) : cart velocity
  // x(3) : pole velocity

  // cart acceleration
  double cart_acceleration = 0.0;
  double theta = z.x(1);
  cart_acceleration = (z.u(0) - params_.friction_cart * z.x(2) +
                       params_.mass_pole * 9.81 * std::cos(theta) * std::sin(theta) +
                       params_.mass_pole * theta * theta * params_.pole_length * std::sin(theta)) /
                      (params_.mass_cart + params_.mass_pole * std::sin(theta) * std::sin(theta));

  // pole acceleration
  double pole_acceleration = 0.0;
  pole_acceleration =
      (-params_.friction_pole * z.x(3) - 9.81 * std::sin(theta) - z.x(2) * std::cos(theta)) /
      params_.pole_length;

  z.x_next(2) = z.x(2) + cart_acceleration * dt_;
  z.x_next(3) = z.x(3) + pole_acceleration * dt_;
  z.x_next(0) = z.x(0) + z.x_next(2) * dt_;
  z.x_next(1) = z.x(1) + z.x_next(3) * dt_;
}

void PoleCart::display_model_params() const {
  std::cout << "Params: " << params_;
  std::cout << std::endl;
}

}  // namespace mppi_est::pole_cart

void operator<<(std::ostream& os, const mppi_est::pole_cart::pole_cart_params& p) {
  os << p.mass_cart << " " << p.mass_pole << " " << p.pole_length << " " << p.friction_cart << " "
     << p.friction_pole << " " << p.cov[0] << " " << p.cov[1] << " " << p.cov[2] << " " << p.cov[3];
}