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
  std::cout << "tank state is " << tank_state << std::endl;
  std::cout << "dt is: " << dt_ << std::endl;

  // I know --- all these numbers make everything messy and bug-prone. This is
  // prototyping, isnt'it?
  double energy =
      u.head<10>().transpose() * x.tail<TORQUE_DIMENSION>().head<10>();

  // 1.414 ~ sqrt(2)
  double delta = std::pow(energy * dt_ / (1.414 * tank_state), 2);
  integration_delta_ += delta;

  constraint_matrix_ = dt_ * x.tail<TORQUE_DIMENSION>().transpose();
  std::cout << "min energy=" << min_energy_ << std::endl;
  std::cout << "integration delta=" << integration_delta_ << std::endl;
  lower_bound_[0] = min_energy_ + integration_delta_ - 0.5 * (tank_state * tank_state);
  std::cout << "Setting lower bound to " << lower_bound_[0] << std::endl;
  t_ = t;
};
