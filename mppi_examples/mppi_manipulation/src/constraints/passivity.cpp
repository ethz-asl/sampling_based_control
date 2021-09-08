//
// Created by giuseppe on 11.08.21.
//

#include "mppi_manipulation/constraints/passivity.h"
#include <iostream>
#include "mppi_manipulation/dimensions.h"

using namespace manipulation;

PassivityConstraint::PassivityConstraint(
    const size_t nx, const double min_energy)
    : ConstraintBase(1, nx), min_energy_(min_energy) {
  first_update_ = true;
  reset(nc_, nx_);
}


void PassivityConstraint::update(const Eigen::VectorXd& x){};

void PassivityConstraint::update_observation(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                        const double t){
  if (first_update_){
    t_ = t;
    first_update_ = false;
  }

  dt_ = t - t_;
  const double tank_state = x(STATE_DIMENSION - TORQUE_DIMENSION - 1);

  std::cout << "torque vector in const matrix: "
            << x.tail<TORQUE_DIMENSION>().head<10>().transpose() << std::endl;
  std::cout << "dt=" << dt_ << std::endl;
  constraint_matrix_ = dt_ * x.tail<TORQUE_DIMENSION>().head<10>().transpose();
  std::cout << "const matrix: " << constraint_matrix_ << std::endl;
  lower_bound_[0] = min_energy_ - 0.5 * (tank_state * tank_state);
  std::cout << "lower bound = " << lower_bound_[0]
            << ", tank_state=" << tank_state << std::endl;
  t_ = t;
};
