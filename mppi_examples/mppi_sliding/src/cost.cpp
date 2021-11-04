/*!
 * @file     pendulum_cart_cost.cpp
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_sliding/cost.h"
#include <ros/package.h>
#include "mppi_sliding/dimensions.h"

using namespace manipulation;

PandaCost::PandaCost(const CostParams& params) : params_(params) {
  robot_model_.init_from_xml(params_.robot_description);
  object_model_.init_from_xml(params_.object_description);
  cylinder_model_.init_from_xml(params_.cylinder_description);
}

mppi::cost_t PandaCost::compute_cost(const mppi::observation_t& x,
                                     const mppi::input_t& u,
                                     const mppi::reference_t& ref,
                                     const double t) {
  double cost = 0.0;
  
  int mode = ref(PandaDim::REFERENCE_DIMENSION - 1);

  robot_model_.update_state(x.head<BASE_ARM_GRIPPER_DIM>());
  //object_model_.update_state(x.segment<1>(2 * BASE_ARM_GRIPPER_DIM));
  cylinder_state_ = Eigen::Vector3d(x(2*BASE_ARM_GRIPPER_DIM),x(2*BASE_ARM_GRIPPER_DIM+1),0);
  //ROS_INFO_STREAM("cylinder state [x,y,fi]: " <<  cylinder_state_);
  //ROS_INFO_STREAM("EE in world frame: " << robot_model_.get_pose(params_.tracked_frame).translation);
  
  // regularization cost
  cost += params_.Qreg *
          x.segment<BASE_ARM_GRIPPER_DIM>(BASE_ARM_GRIPPER_DIM).norm();
  
  // end effector reaching cost
  if (mode == 0) {
    Eigen::Vector3d ref_t = ref.head<3>();
    Eigen::Quaterniond ref_q(ref.segment<4>(3));
    robot_model_.get_error(params_.tracked_frame, ref_q, ref_t, error_);
    cost +=
        (error_.head<3>().transpose() * error_.head<3>()).norm() * params_.Qt;
    cost +=
        (error_.tail<3>().transpose() * error_.tail<3>()).norm() * params_.Qr;

    if (x(2 * BASE_ARM_GRIPPER_DIM + 2 * OBJECT_DIMENSION) > 0) cost += params_.Qc;
  }

  // handle reaching cost
  else if (mode == 1) {
    // error_ = mppi_pinocchio::diff(
    //     robot_model_.get_pose(params_.tracked_frame),
    //     object_model_.get_pose(params_.handle_frame) * params_.grasp_offset);
    error_.resize(3);
    error_(0) = robot_model_.get_pose(params_.tracked_frame).translation(0) - cylinder_state_(0) -0.2;
    error_(1) = robot_model_.get_pose(params_.tracked_frame).translation(1) - cylinder_state_(1);
    error_(2) = robot_model_.get_pose(params_.tracked_frame).translation(2) - 1;
    cost +=
        (error_.head<3>().transpose() * error_.head<3>()).norm() * params_.Qt;
    // cost +=
    //     (error_.tail<3>().transpose() * error_.tail<3>()).norm() * params_.Qr;
    

    // adjust ee pose, when manipulating cylinder
    Eigen::Vector3d ref_t = ref.head<3>();
    Eigen::Quaterniond ref_q(ref.segment<4>(3));
    robot_model_.get_error(params_.tracked_frame, ref_q, ref_t, error_);
    cost +=
        (error_.tail<3>().transpose() * error_.tail<3>()).norm() * params_.Qr;

    // contact cost
    if (x(2*BASE_ARM_GRIPPER_DIM + 2*OBJECT_DIMENSION) > 0) {  //check if this is contanct term
      cost += params_.Qc;
    }
  }

  // object displacement cost
  else if (mode == 2) {
    // error_ = mppi_pinocchio::diff(
    //     robot_model_.get_pose(params_.tracked_frame),
    //     object_model_.get_pose(params_.handle_frame) * params_.grasp_offset);
    error_.resize(3);
    error_(0) = robot_model_.get_pose(params_.tracked_frame).translation(0) - cylinder_state_(0);
    error_(1) = robot_model_.get_pose(params_.tracked_frame).translation(1) - cylinder_state_(1);
    error_(2) = robot_model_.get_pose(params_.tracked_frame).translation(2) - 1;

    cost +=
        (error_.head<3>().transpose() * error_.head<3>()).norm() * params_.Qt2;
    cost +=
        (error_.tail<3>().transpose() * error_.tail<3>()).norm() * params_.Qr2;

    double object_error =
        x(2 * BASE_ARM_GRIPPER_DIM) -
        ref(REFERENCE_POSE_DIMENSION + REFERENCE_OBSTACLE);  // TODO: object position diff 
    std::cout << "object error : " << object_error << std::endl;
    cost += object_error * object_error * params_.Q_obj;
  }
  
  // power cost
  // cost += params_.Q_power * std::max(0.0, (-x.tail<12>().head<10>().transpose() * u.head<10>())(0) - params_.max_power); 
  cost += params_.Q_power * std::max(0.0, (-x.tail<12>().head<10>().transpose() * u.head<10>())(0) - params_.max_power) * 0.1; 
  
  // self collision cost
  robot_model_.get_offset(params_.collision_link_0, params_.collision_link_1,
                          collision_vector_);
  cost += params_.Q_collision * std::pow(std::max(0.0, params_.collision_threshold - collision_vector_.norm()), 2);
  
  // TODO(giuseppe) hard coded for now to match the collision pairs of the safety filter
  robot_model_.get_offset("panda_link0", "panda_link7", collision_vector_);
  cost += params_.Q_collision * std::pow(std::max(0.0, params_.collision_threshold - collision_vector_.norm()), 2);

  // arm reach cost
  double reach;
  robot_model_.get_offset(params_.arm_base_frame, params_.tracked_frame,
                          distance_vector_);
  reach = distance_vector_.head<2>().norm();
  if (reach > params_.max_reach) {
    cost += params_.Q_reach +
            params_.Q_reachs * (std::pow(reach - params_.max_reach, 2));
  }

  if (distance_vector_.norm() < params_.min_dist) {
    cost += params_.Q_reach +
            params_.Q_reachs * (std::pow(reach - params_.min_dist, 2));
  }
  
  // joint limits cost
  for (size_t i = 0; i < 10; i++) {
    if (x(i) < params_.lower_joint_limits[i])
      cost += params_.Q_joint_limit +
              params_.Q_joint_limit_slope *
                  std::pow(params_.lower_joint_limits[i] - x(i), 2);

    if (x(i) > params_.upper_joint_limits[i])
      cost += params_.Q_joint_limit +
              params_.Q_joint_limit_slope *
                  std::pow(x(i) - params_.upper_joint_limits[i], 2);
  }

  return cost;

}
