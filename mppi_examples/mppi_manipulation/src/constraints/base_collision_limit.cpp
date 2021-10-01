//
// Created by giuseppe on 11.08.21.
//

#include "mppi_manipulation/constraints/base_collision_limit.h"

#include <iostream>
#include "mppi_manipulation/dimensions.h"

using namespace manipulation;

BaseCollisionLimit::BaseCollisionLimit(const size_t nx,
                                       const BaseCollsionSettings& settings)
    : ConstraintBase(1, nx), settings_(settings) {
  reset(nc_, nx_);

  min_distance_ = settings_.min_distance;

  object_.init_from_xml(settings_.object_urdf);

  Eigen::VectorXd q = Eigen::VectorXd::Zero(1);
  object_.update_state(q);

  frame_position_ = object_.get_pose(settings_.object_frame_id).translation.head<2>();
}

void BaseCollisionLimit::reset_constraint() {}

void BaseCollisionLimit::update(const Eigen::VectorXd& x){};

void BaseCollisionLimit::update_observation(const Eigen::VectorXd& x,
                                            const Eigen::VectorXd& u,
                                            const double t) {
  // clang-format off
  J_ << std::cos(x(2)), -std::sin(x(2)),
        std::sin(x(2)), std::cos(x(2));
  // clang-format on

  dist_ = x.head<2>() - frame_position_;

  lower_bound_[0] = -0.5 * (dist_.norm() - min_distance_ * min_distance_);
  constraint_matrix_.block(0, 0, 1, 2) = dist_.transpose() * J_;
};

void BaseCollisionLimit::update_violation(const Eigen::VectorXd& x) {
  violation_[0] = std::max(lower_bound_[0], 0.0);
}