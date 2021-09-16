//
// Created by giuseppe on 11.08.21.
//

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <iostream>
#include "mppi_manipulation/constraints/base_collision_limit.h"
#include "mppi_manipulation/dimensions.h"

using namespace manipulation;

BaseCollisionLimit::BaseCollisionLimit(const size_t nx,
                                       const BaseCollsionSettings& settings)
    : ConstraintBase(1, nx), settings_(settings) {
  reset(nc_, nx_);

  min_distance_ = settings_.min_distance;
  pinocchio::urdf::buildModelFromXML(settings_.object_urdf, *model_);
  data_ = std::make_shared<pinocchio::Data>(*model_);
  frame_idx_ = model_->getFrameId(settings_.object_frame_id);
  frame_position_ = data_->oMf[frame_idx_].translation().head<2>();
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
  constraint_matrix_ = dist_.transpose() * J_;
};
