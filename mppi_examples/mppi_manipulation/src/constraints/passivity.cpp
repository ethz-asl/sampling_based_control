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


void PassivityConstraint::reset_constraint() {
  first_update_ = true;
}

void PassivityConstraint::update(const Eigen::VectorXd& x){};

void PassivityConstraint::update_observation(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                        const double t){
  if (first_update_){
    t_ = t;
    first_update_ = false;
    lower_bound_[0] = -1;
    constraint_matrix_.setZero();
  }

  dt_ = t - t_;
  const double tank_state = x(STATE_DIMENSION - TORQUE_DIMENSION - 1);

  // naive approach
  //constraint_matrix_ = dt_ * x.tail<TORQUE_DIMENSION>().head<10>().transpose();
  //lower_bound_[0] = min_energy_ - 0.5 * (tank_state * tank_state);
 
  // using the barrier function approach
  constraint_matrix_ = x.tail<TORQUE_DIMENSION>().head<10>().transpose();
  lower_bound_[0] = min_energy_ - 0.5 * (tank_state * tank_state);
  
  t_ = t;
};
