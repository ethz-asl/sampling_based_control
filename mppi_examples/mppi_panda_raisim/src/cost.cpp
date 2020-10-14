/*!
 * @file     pendulum_cart_cost.cpp
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_panda_raisim/cost.h"

#include <ros/package.h>

using namespace panda;

PandaCost::PandaCost(const std::string& robot_description, double linear_weight, double angular_weight, double obstacle_radius){

  pinocchio::urdf::buildModelFromXML(robot_description, model_);
  data_ = pinocchio::Data(model_);
  frame_id_ = model_.getFrameId(tracked_frame_);
  Q_linear_ = Eigen::Matrix3d::Identity() * linear_weight;
  Q_angular_ = Eigen::Matrix3d::Identity() * angular_weight;
  obstacle_radius_ = obstacle_radius;

  // TODO remove hard coded joint limits
  joint_limits_lower_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
  joint_limits_upper_ << 2.8973, 1.7628, 2.8973, 0.0698, 2.8973, 3.7525, 2.8973;
  std::cout << "Lower joints limits: " << joint_limits_lower_.transpose();
  std::cout << "Upper joints limits: " << joint_limits_upper_.transpose();

}

mppi::CostBase::cost_t PandaCost::compute_cost(const mppi::observation_t& x,
    const mppi::reference_t& ref, const double t) {

  static double linear_cost = 0;
  static double angular_cost = 0;
  double obstacle_cost = 0;
  double joint_limit_cost = 0;

  pose_current_ = get_pose_end_effector(x.head<7>());
  Eigen::Vector3d ref_t = ref.head<3>();
  Eigen::Quaterniond ref_q(ref.segment<4>(3));
  pose_reference_ = pinocchio::SE3(ref_q, ref_t);
  pinocchio::Motion err = pinocchio::log6(pose_current_.actInv(pose_reference_));

  linear_cost = err.linear().transpose() * Q_linear_ * err.linear();
  angular_cost = err.angular().transpose() * Q_angular_ * err.angular();

  double obstacle_dist = (pose_current_.translation() - ref.tail<3>()).norm();
  if (obstacle_dist < obstacle_radius_)
    obstacle_cost =  Q_obst_;

  // joint limits
  for(size_t i=0; i<7; i++){

    if (x(i) < joint_limits_lower_(i))
      joint_limit_cost += 1000 + 100 * std::pow(joint_limits_lower_(i) - x(i), 2);

    if (x(i) > joint_limits_upper_(i))
      joint_limit_cost += 1000 + 100 * std::pow(x(i) - joint_limits_upper_(i), 2);
  }
  return linear_cost + angular_cost + obstacle_cost + joint_limit_cost;

}


pinocchio::SE3 PandaCost::get_pose_end_effector(const Eigen::VectorXd& x){
  pinocchio::forwardKinematics(model_, data_, x.head<7>());
  pinocchio::updateFramePlacements(model_, data_);
  return data_.oMf[frame_id_];
}
