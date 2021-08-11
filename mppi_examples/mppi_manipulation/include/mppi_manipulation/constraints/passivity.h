//
// Created by giuseppe on 11.08.21.
//

#pragma once

#include "safety_filter/constraints/constraint_base.hpp"

namespace manipulation {

class PassivityConstraint : public safety_filter::ConstraintBase {
 public:
  PassivityConstraint(const size_t nx, double min_energy);

 public:
  void update(const Eigen::VectorXd& x) override;
  void update_observation(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                          const double t) override;

 private:
  double t_;
  double dt_;
  double integration_delta_;
  bool first_update_;
  double min_energy_;
};

}  // namespace manipulation
