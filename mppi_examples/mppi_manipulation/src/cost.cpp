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
    cost += (err_.linear().transpose() * err_.linear()).norm() * param_.Qt2;
    cost += (err_.angular().transpose() * err_.angular()).norm() * param_.Qr2;

    // TODO(giuseppe) read the reference value from ref_
    double object_error = x.tail(2 * OBJECT_DIMENSION + CONTACT_STATE).head<1>()(0) -
                         ref(REFERENCE_POSE_DIMENSION + REFERENCE_OBSTACLE);

    // when almost opened reintroduce contact cost to release contact
    cost += object_error * object_error * param_.Q_obj;
  }

  double obstacle_dist = (pose_current_.translation() - ref.segment<3>(7)).norm();
  if (obstacle_dist < param_.ro) cost += param_.Qo;

  // joint limits
  if (!fixed_base_) {
    for (size_t i = 0; i < 7; i++) {
      if (x(i + BASE_DIMENSION) < param_.lower_joint_limits[i])
        cost += param_.Q_joint_limit + param_.Q_joint_limit_slope * std::pow(param_.lower_joint_limits[i] - x(i + BASE_DIMENSION), 2);

      if (x(i + BASE_DIMENSION) > param_.upper_joint_limits[i])
        cost += param_.Q_joint_limit + param_.Q_joint_limit_slope * std::pow(x(i + BASE_DIMENSION) - param_.upper_joint_limits[i], 2);
    }
  }
  else{
    for (size_t i = 0; i < 7; i++) {
      if (x(i) < param_.lower_joint_limits[i])
        cost += param_.Q_joint_limit + param_.Q_joint_limit_slope * std::pow(param_.lower_joint_limits[i] - x(i), 2);

      if (x(i) > param_.upper_joint_limits[i])
        cost += param_.Q_joint_limit + param_.Q_joint_limit_slope * std::pow(x(i) - param_.upper_joint_limits[i], 2);
    }
  }
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

bool PandaCostParam::parse_from_ros(const ros::NodeHandle& nh){
  if (!nh.getParam("obstacle_weight", Qo) || Qo < 0){
    ROS_ERROR("Filed to parse obstacle_weight or invalid!");
    return false;
  }

  if (!nh.getParam("linear_weight", Qt) || Qt < 0){
    ROS_ERROR("Filed to parse linear_weight or invalid!");
    return false;
  }

  if (!nh.getParam("angular_weight", Qr) || Qr < 0){
    ROS_ERROR("Filed to parse angular_weight or invalid!");
    return false;
  }
  
  if (!nh.getParam("contact_weight", Qc) || Qc < 0){
    ROS_ERROR("Filed to parse contact_weight or invalid!");
    return false;
  }

  std::vector<double> trans;
  if (!nh.getParam("grasp_translation_offset", trans) || trans.size() != 3){
    ROS_ERROR("Filed to parse grasp_translation_offset or invalid!");
    return false;
  }

  std::vector<double> rot;
  if (!nh.getParam("grasp_orientation_offset", rot) || rot.size() != 4){
    ROS_ERROR("Filed to parse grasp_orientation_offset or invalid!");
    return false;
  }
  Eigen::Vector3d t(trans[0], trans[1], trans[2]);
  Eigen::Quaterniond q(rot[3], rot[0], rot[1], rot[2]);
  grasp_offset = pinocchio::SE3(q, t);

  if (!nh.getParam("obstacle_radius", ro) || ro < 0){
    ROS_ERROR("Filed to parse obstacle_radius or invalid!");
    return false;
  }

  if (!nh.getParam("upper_joint_limits", upper_joint_limits) || upper_joint_limits.size() != 7){
    ROS_ERROR("Filed to parse upper_joint_limits or invalid!");
    return false;
  }

  if (!nh.getParam("lower_joint_limits", lower_joint_limits) || lower_joint_limits.size() != 7){
    ROS_ERROR("Filed to parse lower_joint_limits or invalid!");
    return false;
  }

  if (!nh.getParam("joint_limit_cost", Q_joint_limit) || Q_joint_limit < 0){
    ROS_ERROR("Filed to parse joint_limit_cost or invalid!");
    return false;
  }

  if (!nh.getParam("joint_limit_slope", Q_joint_limit_slope) || Q_joint_limit_slope < 0){
    ROS_ERROR("Filed to parse joint_limit_slope or invalid!");
    return false;
  }

  if (!nh.getParam("object_weight", Q_obj) || Q_obj < 0){
    ROS_ERROR("Filed to parse object_weight or invalid!");
    return false;
  }

  if (!nh.getParam("object_tolerance", Q_tol) || Q_tol < 0){
    ROS_ERROR("Filed to parse object_tolerance or invalid!");
    return false;
  }

  if (!nh.getParam("linear_weight_opening", Qt2) || Qt2 < 0){
    ROS_ERROR("Filed to parse linear_weight_opening or invalid!");
    return false;
  }

  if (!nh.getParam("angular_weight_opening", Qr2) || Qr2 < 0){
    ROS_ERROR("Filed to parse angular_weight_opening or invalid!");
    return false;
  }

  return true;
}


std::ostream& operator<<(std::ostream& os, const manipulation::PandaCostParam& param){
  // clang-format off
  os << "========================================" << std::endl;
  os << "        Panda Cost Parameters           " << std::endl;
  os << "========================================" << std::endl;
  os << " obstacle_weight: "         << param.Qo << std::endl;
  os << " obstacle_radius: "         << param.ro << std::endl;
  os << " linear_weight: "           << param.Qt << std::endl;
  os << " linear_weight_opening: "   << param.Qt2 << std::endl;
  os << " angular_weight: "          << param.Qr << std::endl;
  os << " angular_weight_opening: "  << param.Qr2 << std::endl;
  os << " contact_weight: "          << param.Qc << std::endl;
  os << " object_weight: "           << param.Q_obj << std::endl;
  os << " object_weight_tolerance: " << param.Q_tol << std::endl;
  os << " grasp offset: "            << param.grasp_offset << std::endl;
  os << " joint limit weight: "       << param.Q_joint_limit << std::endl;
  os << " joint limit slope: "        << param.Q_joint_limit_slope << std::endl;
  os << " upper joint limits: ";
  for (size_t i=0; i<7; i++) os << param.upper_joint_limits[i] << " ";
  os << std::endl;
  os << " lower joint limits: ";
  for (size_t i=0; i<7; i++) os << param.lower_joint_limits[i] << " ";
  os << std::endl;
  os << "========================================" << std::endl;
  // clang-format on  
  
  return os;
}