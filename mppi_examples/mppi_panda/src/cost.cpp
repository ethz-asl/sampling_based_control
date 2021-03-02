/*!
 * @file     pendulum_cart_cost.cpp
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_panda/cost.h"

#include <ros/package.h>

#define PANDA_UPPER_LIMITS \
  2.8973, 1.7628, 2.8973, 0.0698, 2.8973, 3.7525, 2.8973
#define PANDA_LOWER_LIMITS \
  -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973

using namespace panda;

PandaCost::PandaCost(const std::string& robot_description, double linear_weight,
                     double angular_weight, double obstacle_radius) {
  robot_description_ = robot_description;
  linear_weight_ = linear_weight;
  angular_weight_ = angular_weight;
  obstacle_radius_ = obstacle_radius;

  robot_model_.init_from_xml(robot_description_);

  Q_linear_ = Eigen::Matrix3d::Identity() * linear_weight;
  Q_angular_ = Eigen::Matrix3d::Identity() * angular_weight;
  obstacle_radius_ = obstacle_radius;

  // TODO remove hard coded joint limits
  joint_limits_lower_ << PANDA_LOWER_LIMITS;
  joint_limits_upper_ << PANDA_UPPER_LIMITS;
  std::cout << "Lower joints limits: " << joint_limits_lower_.transpose()
            << std::endl;
  std::cout << "Upper joints limits: " << joint_limits_upper_.transpose()
            << std::endl;
}

mppi::CostBase::cost_t PandaCost::compute_cost(const mppi::observation_t& x,
                                               const mppi::reference_t& ref,
                                               const double t) {
  double cost = 0;
  robot_model_.update_state(x.head<7>());

  Eigen::Vector3d ref_t = ref.head<3>();
  Eigen::Quaterniond ref_q(ref.segment<4>(3));
  Eigen::Matrix<double, 6, 1> error;
  robot_model_.get_error("panda_hand", ref_q, ref_t, error);
  cost += error.head<3>().transpose() * Q_linear_ * error.head<3>();
  cost += error.tail<3>().transpose() * Q_angular_ * error.tail<3>();

  double obstacle_dist =
      (robot_model_.get_pose("panda_hand").translation - ref.tail<3>()).norm();
  if (obstacle_dist < obstacle_radius_) cost += Q_obst_;

  // joint limits
  for (size_t i = 0; i < 7; i++) {
    if (x(i) < joint_limits_lower_(i))
      cost += 1000 + 100 * std::pow(joint_limits_lower_(i) - x(i), 2);

    if (x(i) > joint_limits_upper_(i))
      cost += 1000 + 100 * std::pow(x(i) - joint_limits_upper_(i), 2);
  }
  return cost;
}
