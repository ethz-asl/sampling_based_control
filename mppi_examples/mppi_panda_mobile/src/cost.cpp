/*!
 * @file     pendulum_cart_cost.cpp
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "mppi_panda_mobile/cost.h"

#include <math.h>
#include <mppi/cost/cost_base.h>
#include <ros/ros.h>

#include <ros/package.h>

#define PANDA_UPPER_LIMITS 2.8973, 1.7628, 2.8973, 0.0698, 2.8973, 3.7525, 2.8973
#define PANDA_LOWER_LIMITS -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973

namespace panda_mobile {

PandaMobileCost::PandaMobileCost(const std::string& robot_description, const double linear_weight,
                                 const double angular_weight, const double obstacle_radius)
    : robot_description_(robot_description),
      linear_weight_(linear_weight),
      angular_weight_(angular_weight),
      obstacle_radius_(obstacle_radius) {
  pinocchio::urdf::buildModelFromXML(robot_description_, model_);
  data_ = pinocchio::Data(model_);
  frame_id_ = model_.getFrameId(tracked_frame_);

  Q_linear_ = Eigen::Matrix3d::Identity() * linear_weight;
  Q_angular_ = Eigen::Matrix3d::Identity() * angular_weight;

  // TODO remove hard coded joint limits
  joint_limits_lower_ << PANDA_LOWER_LIMITS;
  joint_limits_upper_ << PANDA_UPPER_LIMITS;
}

PandaMobileCost::cost_ptr PandaMobileCost::create() {
  return std::make_shared<PandaMobileCost>(robot_description_, linear_weight_, angular_weight_,
                                           obstacle_radius_);
}

PandaMobileCost::cost_ptr PandaMobileCost::clone() const {
  return std::make_shared<PandaMobileCost>(*this);
}

void PandaMobileCost::set_linear_weight(const double k) { Q_linear_ *= k; }

void PandaMobileCost::set_angular_weight(const double k) { Q_angular_ *= k; }

void PandaMobileCost::set_obstacle_radius(const double r) { obstacle_radius_ = r; }

pinocchio::SE3 PandaMobileCost::get_current_pose(const Eigen::VectorXd& x) {
  pinocchio::forwardKinematics(model_, data_, x.head<7>());
  pinocchio::updateFramePlacements(model_, data_);
  Eigen::Matrix3d base_rotation(Eigen::AngleAxisd(x(9), Eigen::Vector3d::UnitZ()));
  Eigen::Vector3d base_translation(x(7), x(8), 0.0);
  pinocchio::SE3 base_tf = pinocchio::SE3(base_rotation, base_translation);
  return base_tf.act(data_.oMf[frame_id_]);
}

PandaMobileCost::cost_t PandaMobileCost::compute_cost(const mppi::observation_t& x,
                                                      const mppi::reference_t& ref,
                                                      const double t) {
  cost_t cost;

  // target reaching cost
  pose_current_ = get_current_pose(x);
  Eigen::Vector3d ref_t = ref.head<3>();
  Eigen::Quaterniond ref_q(ref.segment<4>(3));
  pose_reference_ = pinocchio::SE3(ref_q, ref_t);
  pinocchio::Motion err = pinocchio::log6(pose_current_.actInv(pose_reference_));
  cost += err.linear().transpose() * Q_linear_ * err.linear();
  cost += err.angular().transpose() * Q_angular_ * err.angular();

  // obstacle avoidance cost
  double obstacle_dist = (pose_current_.translation() - ref.tail<3>()).norm();
  if (obstacle_dist < obstacle_radius_)
    cost += Q_obst_;

  // reach cost
  if (data_.oMf[frame_id_].translation().head<2>().norm() > 1.0)
    cost += Q_reach_;

  // joint limits cost
  for(size_t i=0; i<7; i++){
    if (x(i) < joint_limits_lower_(i))
      cost += 100 + 10 * std::pow(joint_limits_lower_(i) - x(i), 2);
    if (x(i) > joint_limits_upper_(i))
      cost += 100 + 10 * std::pow(x(i) - joint_limits_upper_(i), 2);
  }

  return cost;
}

}  // namespace panda_mobile