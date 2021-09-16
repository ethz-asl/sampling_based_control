//
// Created by giuseppe on 11.08.21.
//

#pragma once

#include <memory>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include "safety_filter/constraints/constraint_base.hpp"

namespace manipulation {

struct BaseCollsionSettings {
  std::string object_frame_id;
  std::string object_urdf;
  double min_distance;
};

class BaseCollisionLimit : public safety_filter::ConstraintBase {
 public:
  BaseCollisionLimit(const size_t nx, const BaseCollsionSettings& settings);

 public:
  void update(const Eigen::VectorXd& x) override;
  void update_observation(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                          const double t) override;
  void reset_constraint() override;

 private:
  BaseCollsionSettings settings_;

  int frame_idx_;
  Eigen::Vector2d dist_;
  Eigen::Vector2d frame_position_;
  Eigen::Matrix2d J_;
  double min_distance_;

  // unique ptr requires a complete type --> cannot be used with forward
  // declaration
  std::shared_ptr<pinocchio::Model> model_;
  std::shared_ptr<pinocchio::Data> data_;
};

}  // namespace manipulation
