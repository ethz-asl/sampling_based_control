//
// Created by giuseppe on 11.08.21.
//

#include "mppi_manipulation/constraints/passivity.h"
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

  dt_ = t_ - t;
  const double& tank_state = x(STATE_DIMENSION-1);
  double energy = u.head<BASE_ARM_GRIPPER_DIM>().transpose() * u.tail<BASE_ARM_GRIPPER_DIM>();

  double delta = std::pow(energy * dt_ / (std::sqrt(2) * tank_state), 2);
  integration_delta_ += delta;

  constraint_matrix_ = dt_ * u.tail<TORQUE_DIMENSION>().transpose();
  lower_bound_[0] = min_energy_ + integration_delta_ - 0.5 * (tank_state * tank_state);
  t_ = t;
};
