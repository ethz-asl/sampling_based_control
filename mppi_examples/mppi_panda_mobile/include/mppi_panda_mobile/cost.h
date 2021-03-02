/*!
 * @file     pendulum_cart_cost.h
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#pragma once
#include <mppi_pinocchio/model.h>

#include <mppi/cost/cost_base.h>
#include <ros/ros.h>

#include <ros/package.h>

namespace panda_mobile {

class PandaMobileCost : public mppi::CostBase {
 public:
  using cost_ptr = mppi::CostBase::cost_ptr;

  PandaMobileCost(const std::string& robot_description,
                  const double linear_weight, const double angular_weight,
                  const double obstacle_radius, bool joint_limits = false);
  ~PandaMobileCost() = default;

 private:
  bool joint_limits_;
  double linear_weight_;
  double angular_weight_;
  double obstacle_radius_;

  std::string robot_description_;
  mppi_pinocchio::RobotModel robot_model_;
  std::string tracked_frame_ = "panda_hand";

  Eigen::Matrix<double, 3, 3> Q_linear_;
  Eigen::Matrix<double, 3, 3> Q_angular_;

  double Q_obst_ = 100000;
  double Q_reach_ = 100000;
  bool obstacle_set_ = false;

  Eigen::Matrix<double, 7, 1> joint_limits_lower_;
  Eigen::Matrix<double, 7, 1> joint_limits_upper_;

 public:
  cost_ptr create() override;
  cost_ptr clone() const override;

  void set_linear_weight(const double k);
  void set_angular_weight(const double k);
  void set_obstacle_radius(const double r);
  mppi_pinocchio::Pose get_current_pose(const Eigen::VectorXd& x);
  cost_t compute_cost(const mppi::observation_t& x,
                      const mppi::reference_t& ref, const double t) override;
};
}  // namespace panda_mobile
