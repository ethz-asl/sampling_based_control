/*!
 * @file     pendulum_cart_cost.cpp
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_manipulation/cost.h"
#include <ros/package.h>
#include "mppi_manipulation/dimensions.h"

#define LOWER_LIMITS -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973
#define UPPER_LIMITS 2.8973, 1.7628, 2.8973, 0.0698, 2.8973, 3.7525, 2.8973

// TODO(giuseppe) remove the gripper cost and hand code the gripper positions
using namespace manipulation;

PandaCost::PandaCost(const std::string& robot_description, const std::string& object_description,
                     const PandaCostParam& param, bool fixed_base)
    : fixed_base_(fixed_base) {
  robot_description_ = robot_description;
  object_description_ = object_description;
  param_ = param;

  // robot model
  pinocchio::urdf::buildModelFromXML(robot_description, model_);
  data_ = pinocchio::Data(model_);
  frame_id_ = model_.getFrameId(tracked_frame_);

  // object model
  pinocchio::urdf::buildModelFromXML(object_description, object_model_);
  object_data_ = pinocchio::Data(object_model_);
  handle_idx_ = object_model_.getFrameId("handle_link");

  // TODO(giuseppe) remove hard coded joint limits
  joint_limits_lower_ << LOWER_LIMITS;
  joint_limits_upper_ << UPPER_LIMITS;
}

mppi::CostBase::cost_t PandaCost::compute_cost(const mppi::observation_t& x,
                                               const mppi::reference_t& ref, const double t) {
  double cost = 0.0;
  pose_current_ = get_pose_end_effector(x);

  // end effector reaching
  int mode = ref(PandaDim::REFERENCE_DIMENSION - 1);

  // end effector reaching task
  if (mode == 0) {
    Eigen::Vector3d ref_t = ref.head<3>();
    Eigen::Quaterniond ref_q(ref.segment<4>(3));
    pose_reference_ = pinocchio::SE3(ref_q, ref_t);
    err_ = pinocchio::log6(pose_current_.actInv(pose_reference_));
    cost += (err_.linear().transpose() * err_.linear()).norm() * param_.Qt;
    cost += (err_.angular().transpose() * err_.angular()).norm() * param_.Qr;

    if (x.tail<1>()(0) > 0) cost += param_.Qc;
  }
  // reach the handle with open gripper
  else if (mode == 1) {
    pose_handle_ = get_pose_handle(x);
    err_ = pinocchio::log6(pose_current_.actInv(pose_handle_.act(param_.grasp_offset)));
    cost += (err_.linear().transpose() * err_.linear()).norm() * param_.Qt;
    cost += (err_.angular().transpose() * err_.angular()).norm() * param_.Qr;

    if (x.tail<1>()(0) > 0) cost += param_.Qc;
  }
  // keep only position control in proximity of the handle and no gripper cost
  // move the object
  else if (mode == 2) {
    pose_handle_ = get_pose_handle(x);
    err_ = pinocchio::log6(pose_current_.actInv(pose_handle_.act(param_.grasp_offset)));
    cost += (err_.linear().transpose() * err_.linear()).norm() * param_.Qt / 100.0;

    // TODO(giuseppe) read the reference value from ref_
    cost += std::pow(x.tail(2 * OBJECT_DIMENSION + CONTACT_STATE).head<1>()(0) -
                         ref(REFERENCE_POSE_DIMENSION + REFERENCE_OBSTACLE),
                     2) * 10;
  }

  double obstacle_dist = (pose_current_.translation() - ref.segment<3>(7)).norm();
  if (obstacle_dist < param_.ro) cost += param_.Qo;

  // joint limits
  /**
  for(size_t i=0; i<7; i++){

    if (x(i) < joint_limits_lower_(i))
      joint_limit_cost += 1000 + 100 * std::pow(joint_limits_lower_(i) - x(i),
  2);

    if (x(i) > joint_limits_upper_(i))
      joint_limit_cost += 1000 + 100 * std::pow(x(i) - joint_limits_upper_(i),
  2);
  }
   **/
  return cost;
}

pinocchio::SE3 PandaCost::get_pose_end_effector(const Eigen::VectorXd& x) {
  if (fixed_base_) {
    pinocchio::forwardKinematics(model_, data_, x.head<ARM_GRIPPER_DIM>());
  } else {
    pinocchio::forwardKinematics(model_, data_, x.head<BASE_ARM_GRIPPER_DIM>());
  }
  pinocchio::updateFramePlacements(model_, data_);
  return data_.oMf[frame_id_];
}

pinocchio::SE3 PandaCost::get_pose_handle(const Eigen::VectorXd& x) {
  pinocchio::forwardKinematics(object_model_, object_data_,
                               x.tail<2 * OBJECT_DIMENSION + CONTACT_STATE>().head<1>());
  pinocchio::updateFramePlacements(object_model_, object_data_);
  return object_data_.oMf[handle_idx_];
}
