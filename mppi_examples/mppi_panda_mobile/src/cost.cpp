/*!
 * @file     pendulum_cart_cost.cpp
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_panda_mobile/cost.h"

#include <ros/ros.h>
#include <cmath>

#include <ros/package.h>

#define PANDA_UPPER_LIMITS \
  2.0, 2.0, 6.28, 2.8973, 1.7628, 2.8973, 0.0698, 2.8973, 3.7525, 2.8973
#define PANDA_LOWER_LIMITS                                                 \
  -2.0, -2.0, -6.28, -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, \
      -2.8973

namespace panda_mobile {

PandaMobileCost::PandaMobileCost(const std::string& robot_description,
                                 const double linear_weight,
                                 const double angular_weight,
                                 const double obstacle_radius,
                                 bool joint_limits)
    : robot_description_(robot_description),
      linear_weight_(linear_weight),
      angular_weight_(angular_weight),
      obstacle_radius_(obstacle_radius),
      joint_limits_(joint_limits) {
  robot_model_.init_from_xml(robot_description);

  Q_linear_ = Eigen::Matrix3d::Identity() * linear_weight;
  Q_angular_ = Eigen::Matrix3d::Identity() * angular_weight;

  // TODO remove hard coded joint limits
  joint_limits_lower_ << PANDA_LOWER_LIMITS;
  joint_limits_upper_ << PANDA_UPPER_LIMITS;
}

mppi::cost_ptr PandaMobileCost::create() {
  return std::make_shared<PandaMobileCost>(robot_description_, linear_weight_,
                                           angular_weight_, obstacle_radius_,
                                           joint_limits_);
}

mppi::cost_ptr PandaMobileCost::clone() const {
  return std::make_shared<PandaMobileCost>(*this);
}

void PandaMobileCost::set_linear_weight(const double k) { Q_linear_ *= k; }

void PandaMobileCost::set_angular_weight(const double k) { Q_angular_ *= k; }

void PandaMobileCost::set_obstacle_radius(const double r) {
  obstacle_radius_ = r;
}

mppi_pinocchio::Pose PandaMobileCost::get_current_pose(
    const Eigen::VectorXd& x) {
  return robot_model_.get_pose(tracked_frame_);
}

mppi::cost_t PandaMobileCost::compute_cost(const mppi::observation_t& x,
                                           const mppi::input_t& u,
                                           const mppi::reference_t& ref,
                                           const double t) {
  mppi::cost_t cost;

  // update model
  robot_model_.update_state(x);

  // target reaching cost

  Eigen::Vector3d ref_t = ref.head<3>();
  Eigen::Quaterniond ref_q(ref.segment<4>(3));
  Eigen::Matrix<double, 6, 1> error;

  mppi_pinocchio::Pose current_pose = get_current_pose(x);
  mppi_pinocchio::Pose reference_pose(ref_t, ref_q);
  error = mppi_pinocchio::diff(current_pose, reference_pose);
  cost += error.head<3>().transpose() * Q_linear_ * error.head<3>();
  cost += error.tail<3>().transpose() * Q_angular_ * error.tail<3>();

  if (cost < 0.001) cost += 10.0 * u.norm();

  // obstacle avoidance cost
  double obstacle_dist = (current_pose.translation - ref.tail<3>()).norm();
  if (obstacle_dist < obstacle_radius_) cost += Q_obst_;

  // reach cost
  if (Q_reach_ > 0) {
    double reach = (robot_model_.get_pose(tracked_frame_).translation -
                    robot_model_.get_pose("panda_link0").translation)
                       .head<2>()
                       .norm();
    if (reach > 1.0) {
      cost += Q_reach_;
    }
  }

  // joint limits cost
  if (joint_limits_) {
    for (int i = 0; i < x.size(); i++) {
      if (x(i) < joint_limits_lower_(i))
        cost += 100 + 10 * std::pow(joint_limits_lower_(i) - x(i), 2);
      if (x(i) > joint_limits_upper_(i))
        cost += 100 + 10 * std::pow(x(i) - joint_limits_upper_(i), 2);
    }
  }

  return cost;
}

}  // namespace panda_mobile