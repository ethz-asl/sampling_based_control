//
// Created by giuseppe on 12/16/20.
//

#include "point_mass.h"

namespace mppi_est::point_mass {

PointMass::PointMass(const point_mass_params& params) : Model(2, 1), params_(params) {
  vector_t cov;
  cov.resize(2);
  cov << params_.pos_cov, params_.vel_cov;
  set_measurement_covariance(cov);
}

void PointMass::reset(const vector_t& x) {
  assert(x.size() == get_measurement_dimension());
}

void PointMass::step(const transition_tuple_t& z) {
  assert(z.u.size() == get_action_dimension());
  assert(z.x.size() == get_measurement_dimension());
  assert(z.x_next.size() == get_measurement_dimension());
  z.x_next(0) = z.x(0) + z.x(1) * dt_;
  z.x_next(1) = z.x(1) + (z.u(0) - params_.mu * z.x(1)) * dt_;
}

}  // namespace mppi_est::test