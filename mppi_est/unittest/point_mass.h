//
// Created by giuseppe on 12/16/20.
//

#pragma once
#include "mppi_est/model.hpp"

namespace mppi_est::test {

struct point_mass_params {
  double mu = 1.0;
  double pos_cov = 0.01;
  double vel_cov = 0.01;
};

class PointMass : public Model {
 public:
  PointMass(const point_mass_params& params);
  std::unique_ptr<Model> create() override { return std::make_unique<PointMass>(params_); }
  void reset(const vector_t& x) override;
  void step(const transition_tuple_t& z) override;


 private:
  double dt_ = 0.01;
  point_mass_params params_;
};

}  // namespace mppi_est::test