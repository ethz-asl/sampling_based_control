//
// Created by giuseppe on 11.08.21.
//

#pragma once

#include <mppi_pinocchio/model.h>

#include <memory>
#include "safety_filter/constraints/constraint_base.hpp"

namespace manipulation {

struct ObstacleCollisionSettings {
  Eigen::VVector3d oobstacle_position;
  double objstacle_radius;
  std::string object_urdf;
};

class ObstacleCollisionLimit : public safety_filter::ConstraintBase {
 public:
  ObstacleCollisionLimit(const size_t nx,
                         const ObstacleCollisionSettings& settings);

 public:
  void update(const Eigen::VectorXd& x) override;
  void update_observation(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                          const double t) override;
  void reset_constraint() override;
  void update_violation(const Eigen::VectorXd& x) override;

 private:
  ObstacleCollisionSettings settings_;

  Eigen::Vector2d dist_;
  Eigen::Vector2d frame_position_;
  Eigen::Matrix2d J_;
  double min_distance_;

  mppi_pinocchio::RobotModel robot_;
};

}  // namespace manipulation
