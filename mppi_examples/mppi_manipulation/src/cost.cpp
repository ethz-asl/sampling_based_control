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

PandaCost::PandaCost(const CostParams& params) : params_(params) {
  robot_model_.init_from_xml(params_.robot_description);
  object_model_.init_from_xml(params_.object_description);
}

mppi::cost_t PandaCost::compute_cost(const mppi::observation_t& x,
                                     const mppi::input_t& u,
                                     const mppi::reference_t& ref,
                                     const double t) {
  double cost = 0.0;
  
  int mode = ref(PandaDim::REFERENCE_DIMENSION - 1);

  robot_model_.update_state(x.head<BASE_ARM_GRIPPER_DIM>());
  object_model_.update_state(x.segment<1>(2 * BASE_ARM_GRIPPER_DIM));

  // regularization cost
  double reg_cost = params_.Qreg * x.segment<BASE_ARM_GRIPPER_DIM>(BASE_ARM_GRIPPER_DIM).norm();
  double pose_cost = 0.0;
  double contact_cost = 0.0;
  double object_cost = 0.0;
  double self_collision_cost = 0.0;
  double power_cost = 0.0;
  double reach_cost = 0.0;
  double joint_limits_cost = 0.0;

  // end effector reaching cost
  if (mode == 0) {
    Eigen::Vector3d ref_t = ref.head<3>();
    Eigen::Quaterniond ref_q(ref.segment<4>(3));
    robot_model_.get_error(params_.tracked_frame, ref_q, ref_t, error_);
    pose_cost += (error_.head<3>().transpose() * error_.head<3>()).norm() * params_.Qt;
    pose_cost += (error_.tail<3>().transpose() * error_.tail<3>()).norm() * params_.Qr;
    // contact cost
    if (x(2 * BASE_ARM_GRIPPER_DIM + 2 * OBJECT_DIMENSION) > 0) contact_cost += params_.Qc;
  }

  // handle reaching cost
  // this cost enforces the offset between the tracked_frame and the handle_frame
  // to be equal to the chosen reference
  else if (mode == 1) {
    // get the desired offset from reference and not from configs
    Eigen::Vector3d ref_t = ref.head<3>();
    Eigen::Quaterniond ref_q(ref.segment<4>(3));
    auto offset = mppi_pinocchio::Pose(ref_t, ref_q);
    error_ = mppi_pinocchio::diff(
        robot_model_.get_pose(params_.tracked_frame),
        object_model_.get_pose(params_.handle_frame) * offset);
    pose_cost += (error_.head<3>().transpose() * error_.head<3>()).norm() * params_.Qt;
    pose_cost += (error_.tail<3>().transpose() * error_.tail<3>()).norm() * params_.Qr;
    // contact cost
    if (x(2 * BASE_ARM_GRIPPER_DIM + 2 * OBJECT_DIMENSION) > 0) contact_cost += params_.Qc;
  }

  // object displacement cost
  else if (mode == 2 || mode == 4) {
    // get the desired offset from reference and not from configs (see mode 1 for details)
    Eigen::Vector3d ref_t = ref.head<3>();
    Eigen::Quaterniond ref_q(ref.segment<4>(3));
    auto offset = mppi_pinocchio::Pose(ref_t, ref_q);
    error_ = mppi_pinocchio::diff(
        robot_model_.get_pose(params_.tracked_frame),
        object_model_.get_pose(params_.handle_frame) * offset);
    pose_cost += (error_.head<3>().transpose() * error_.head<3>()).norm() * params_.Qt2;
    pose_cost += (error_.tail<3>().transpose() * error_.tail<3>()).norm() * params_.Qr2;
    double object_error =
        x(2 * BASE_ARM_GRIPPER_DIM) -
        ref(REFERENCE_POSE_DIMENSION + REFERENCE_OBSTACLE);
    object_cost += object_error * object_error * params_.Q_obj;

    if (mode == 4){
      // mode 4 adds a collision cost if we are not interacting with the hand or finger links
      float hand_force = x.tail<3>().norm();
      if (x(2 * BASE_ARM_GRIPPER_DIM + 2 * OBJECT_DIMENSION) > 0 && hand_force == 0.0){
        // in contact AND there is no external torque at or below the hand joint
        contact_cost += params_.Q_collision;
      }
    }
  }

  // joint-level control
  else if (mode == 3) {
    Eigen::VectorXd err_j = ref.head<7>() - x.segment<7>(3);
    Eigen::VectorXd err_bp = ref.segment<2>(7) - x.head<2>();
    double err_bo = ref[9] - x[2];

    pose_cost += err_j.norm() * params_.Q_j + err_bp.norm() * params_.Q_bp + err_bo * err_bo * params_.Q_bo;
    // contact cost
    if (x(2 * BASE_ARM_GRIPPER_DIM + 2 * OBJECT_DIMENSION) > 0) contact_cost += params_.Qc;
  }
  
  // power cost
  power_cost += params_.Q_power * std::max(0.0, (-x.tail<12>().head<10>().transpose() * u.head<10>())(0) - params_.max_power);
  
  // self collision cost
  robot_model_.get_offset(params_.collision_link_0, params_.collision_link_1,collision_vector_);
  self_collision_cost += params_.Q_collision * std::pow(std::max(0.0, params_.collision_threshold - collision_vector_.norm()), 2);
  
  // TODO(giuseppe) hard coded for now to match the collision pairs of the safety filter
  robot_model_.get_offset("panda_link0", "panda_link7", collision_vector_);
  self_collision_cost += params_.Q_collision * std::pow(std::max(0.0, params_.collision_threshold - collision_vector_.norm()), 2);

  // arm reach cost
  double reach;
  robot_model_.get_offset(params_.arm_base_frame, params_.tracked_frame,
                          distance_vector_);
  reach = distance_vector_.head<2>().norm();
  if (reach > params_.max_reach) {
    reach_cost += params_.Q_reach +
                  params_.Q_reachs * (std::pow(reach - params_.max_reach, 2));
  }

  if (distance_vector_.norm() < params_.min_dist) {
    reach_cost += params_.Q_reach +
                  params_.Q_reachs * (std::pow(reach - params_.min_dist, 2));
  }
  
  // joint limits cost
  for (size_t i = 0; i < 10; i++) {
    if (x(i) < params_.lower_joint_limits[i])
      joint_limits_cost += params_.Q_joint_limit +
                           params_.Q_joint_limit_slope *
                           std::pow(params_.lower_joint_limits[i] - x(i), 2);

    if (x(i) > params_.upper_joint_limits[i])
      joint_limits_cost += params_.Q_joint_limit +
                           params_.Q_joint_limit_slope *
                           std::pow(x(i) - params_.upper_joint_limits[i], 2);
  }

  cost_map_ = { {"reg_cost", reg_cost},
      {"pose_cost", pose_cost}, {"contact_cost", contact_cost},
      {"object_cost", object_cost}, {"self_collision_cost", self_collision_cost},
      {"power_cost", power_cost}, {"reach_cost", reach_cost},
      {"joint_limits_cost", joint_limits_cost},};

  cost = reg_cost + pose_cost + contact_cost + object_cost + self_collision_cost + power_cost + reach_cost + joint_limits_cost;
  return cost;
}
