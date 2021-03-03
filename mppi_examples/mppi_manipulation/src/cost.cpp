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

using namespace manipulation;

PandaCost::PandaCost(const std::string& robot_description,
                     const std::string& object_description,
                     const PandaCostParam& param, bool fixed_base)
    : fixed_base_(fixed_base) {
  robot_description_ = robot_description;
  object_description_ = object_description;
  param_ = param;

  // robot model
  robot_model_.init_from_xml(robot_description_);
  object_model_.init_from_xml(object_description_);
}

mppi::CostBase::cost_t PandaCost::compute_cost(const mppi::observation_t& x,
                                               const mppi::reference_t& ref,
                                               const double t) {
  double cost = 0.0;

  if (fixed_base_) {
    robot_model_.update_state(x.head<ARM_GRIPPER_DIM>());
  } else {
    robot_model_.update_state(x.head<BASE_ARM_GRIPPER_DIM>());
  }

  // end effector reaching
  int mode = ref(PandaDim::REFERENCE_DIMENSION - 1);

  // end effector reaching task
  if (mode == 0) {
    Eigen::Vector3d ref_t = ref.head<3>();
    Eigen::Quaterniond ref_q(ref.segment<4>(3));
    robot_model_.get_error(tracked_frame_, ref_q, ref_t, error_);
    cost +=
        (error_.head<3>().transpose() * error_.head<3>()).norm() * param_.Qt;
    cost +=
        (error_.tail<3>().transpose() * error_.tail<3>()).norm() * param_.Qr;

    if (x.tail<1>()(0) > 0) cost += param_.Qc;
  }
  // reach the handle with open gripper
  else if (mode == 1) {
    object_model_.update_state(
        x.tail<2 * OBJECT_DIMENSION + CONTACT_STATE>().head<1>());
    error_ = mppi_pinocchio::diff(
        robot_model_.get_pose(tracked_frame_),
        object_model_.get_pose(handle_frame_) * param_.grasp_offset);
    cost +=
        (error_.head<3>().transpose() * error_.head<3>()).norm() * param_.Qt;
    cost +=
        (error_.tail<3>().transpose() * error_.tail<3>()).norm() * param_.Qr;

    if (x.tail<1>()(0) > 0) cost += param_.Qc;
  }
  // keep only position control in proximity of the handle and no gripper cost
  // move the object
  else if (mode == 2) {
    object_model_.update_state(
        x.tail<2 * OBJECT_DIMENSION + CONTACT_STATE>().head<1>());
    error_ = mppi_pinocchio::diff(
        robot_model_.get_pose(tracked_frame_),
        object_model_.get_pose(handle_frame_) * param_.grasp_offset);
    cost +=
        (error_.head<3>().transpose() * error_.head<3>()).norm() * param_.Qt2;
    cost +=
        (error_.tail<3>().transpose() * error_.tail<3>()).norm() * param_.Qr2;

    double object_error =
        x.tail(2 * OBJECT_DIMENSION + CONTACT_STATE).head<1>()(0) -
        ref(REFERENCE_POSE_DIMENSION + REFERENCE_OBSTACLE);

    // when almost opened reintroduce contact cost to release contact
    cost += object_error * object_error * param_.Q_obj;
  }

  // obstacle 2d distance projected on base plane
  if (!fixed_base_) {
    double obstacle_dist = (x.head<2>() - ref.segment<2>(7)).norm();
    if (obstacle_dist < param_.ro) {
      cost += param_.Qo + param_.Qos * (param_.ro - obstacle_dist);
    }
  }

  // 2d reach computation
  double reach;
  if (fixed_base_) {
    reach = robot_model_.get_pose(tracked_frame_).translation.head<2>().norm();
  } else {
    robot_model_.get_error(arm_base_frame_, tracked_frame_, error_);
    reach = error_.head<2>().norm();
  }
  if (reach > param_.max_reach) {
    cost += param_.Q_reach +
            param_.Q_reachs * (std::pow(reach - param_.max_reach, 2));
  }

  // joint limits
  if (!fixed_base_) {
    for (size_t i = 0; i < 7; i++) {
      if (x(i + BASE_DIMENSION) < param_.lower_joint_limits[i])
        cost +=
            param_.Q_joint_limit +
            param_.Q_joint_limit_slope *
                std::pow(param_.lower_joint_limits[i] - x(i + BASE_DIMENSION),
                         2);

      if (x(i + BASE_DIMENSION) > param_.upper_joint_limits[i])
        cost +=
            param_.Q_joint_limit +
            param_.Q_joint_limit_slope *
                std::pow(x(i + BASE_DIMENSION) - param_.upper_joint_limits[i],
                         2);
    }
  } else {
    for (size_t i = 0; i < 7; i++) {
      if (x(i) < param_.lower_joint_limits[i])
        cost += param_.Q_joint_limit +
                param_.Q_joint_limit_slope *
                    std::pow(param_.lower_joint_limits[i] - x(i), 2);

      if (x(i) > param_.upper_joint_limits[i])
        cost += param_.Q_joint_limit +
                param_.Q_joint_limit_slope *
                    std::pow(x(i) - param_.upper_joint_limits[i], 2);
    }
  }
  return cost;
}

bool PandaCostParam::parse_from_ros(const ros::NodeHandle& nh) {
  if (!nh.getParam("obstacle_weight", Qo) || Qo < 0) {
    ROS_ERROR("Filed to parse obstacle_weight or invalid!");
    return false;
  }

  if (!nh.getParam("obstacle_weight_slope", Qos) || Qos < 0) {
    ROS_ERROR("Filed to parse obstacle_weight_slope or invalid!");
    return false;
  }

  if (!nh.getParam("linear_weight", Qt) || Qt < 0) {
    ROS_ERROR("Filed to parse linear_weight or invalid!");
    return false;
  }

  if (!nh.getParam("angular_weight", Qr) || Qr < 0) {
    ROS_ERROR("Filed to parse angular_weight or invalid!");
    return false;
  }

  if (!nh.getParam("contact_weight", Qc) || Qc < 0) {
    ROS_ERROR("Filed to parse contact_weight or invalid!");
    return false;
  }

  std::vector<double> trans;
  if (!nh.getParam("grasp_translation_offset", trans) || trans.size() != 3) {
    ROS_ERROR("Filed to parse grasp_translation_offset or invalid!");
    return false;
  }

  std::vector<double> rot;
  if (!nh.getParam("grasp_orientation_offset", rot) || rot.size() != 4) {
    ROS_ERROR("Filed to parse grasp_orientation_offset or invalid!");
    return false;
  }
  Eigen::Vector3d t(trans[0], trans[1], trans[2]);
  Eigen::Quaterniond q(rot[3], rot[0], rot[1], rot[2]);
  grasp_offset = mppi_pinocchio::Pose(t, q);

  if (!nh.getParam("obstacle_radius", ro) || ro < 0) {
    ROS_ERROR("Filed to parse obstacle_radius or invalid!");
    return false;
  }

  if (!nh.getParam("max_reach", max_reach) || max_reach < 0) {
    ROS_ERROR("Filed to parse max_reach or invalid!");
    return false;
  }

  if (!nh.getParam("reach_weight", Q_reach) || Q_reach < 0) {
    ROS_ERROR("Filed to parse reach_weight or invalid!");
    return false;
  }

  if (!nh.getParam("reach_weight_slope", Q_reachs) || Q_reachs < 0) {
    ROS_ERROR("Filed to parse reach_weight_slope or invalid!");
    return false;
  }

  if (!nh.getParam("upper_joint_limits", upper_joint_limits) ||
      upper_joint_limits.size() != 7) {
    ROS_ERROR("Filed to parse upper_joint_limits or invalid!");
    return false;
  }

  if (!nh.getParam("lower_joint_limits", lower_joint_limits) ||
      lower_joint_limits.size() != 7) {
    ROS_ERROR("Filed to parse lower_joint_limits or invalid!");
    return false;
  }

  if (!nh.getParam("joint_limit_cost", Q_joint_limit) || Q_joint_limit < 0) {
    ROS_ERROR("Filed to parse joint_limit_cost or invalid!");
    return false;
  }

  if (!nh.getParam("joint_limit_slope", Q_joint_limit_slope) ||
      Q_joint_limit_slope < 0) {
    ROS_ERROR("Filed to parse joint_limit_slope or invalid!");
    return false;
  }

  if (!nh.getParam("object_weight", Q_obj) || Q_obj < 0) {
    ROS_ERROR("Filed to parse object_weight or invalid!");
    return false;
  }

  if (!nh.getParam("object_tolerance", Q_tol) || Q_tol < 0) {
    ROS_ERROR("Filed to parse object_tolerance or invalid!");
    return false;
  }

  if (!nh.getParam("linear_weight_opening", Qt2) || Qt2 < 0) {
    ROS_ERROR("Filed to parse linear_weight_opening or invalid!");
    return false;
  }

  if (!nh.getParam("angular_weight_opening", Qr2) || Qr2 < 0) {
    ROS_ERROR("Filed to parse angular_weight_opening or invalid!");
    return false;
  }

  return true;
}

std::ostream& operator<<(std::ostream& os,
                         const manipulation::PandaCostParam& param) {
  // clang-format off
  os << "========================================" << std::endl;
  os << "        Panda Cost Parameters           " << std::endl;
  os << "========================================" << std::endl;
  os << " obstacle_weight: "         << param.Qo << std::endl;
  os << " obstacle_radius: "         << param.ro << std::endl;
  os << " obstacle_weight_slope: "   << param.Qos << std::endl;
  os << " linear_weight: "           << param.Qt << std::endl;
  os << " linear_weight_opening: "   << param.Qt2 << std::endl;
  os << " angular_weight: "          << param.Qr << std::endl;
  os << " angular_weight_opening: "  << param.Qr2 << std::endl;
  os << " contact_weight: "          << param.Qc << std::endl;
  os << " reach weight: "            << param.Q_reach << std::endl;
  os << " reach weight slope: "      << param.Q_reachs << std::endl;
  os << " object_weight: "           << param.Q_obj << std::endl;
  os << " object_weight_tolerance: " << param.Q_tol << std::endl;
  os << " grasp offset: "            << std::endl;
  os << " >> translation = "         << param.grasp_offset.translation.transpose() << std::endl;
  os << " >> rotation = "            << param.grasp_offset.rotation.coeffs().transpose() << std::endl;
  os << " joint limit weight: "      << param.Q_joint_limit << std::endl;
  os << " joint limit slope: "       << param.Q_joint_limit_slope << std::endl;
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