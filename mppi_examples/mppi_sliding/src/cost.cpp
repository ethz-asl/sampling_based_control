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
}

mppi::cost_t PandaCost::compute_cost(const mppi::observation_t& x,
                                     const mppi::input_t& u,
                                     const mppi::reference_t& ref,
                                     const double t) {
  double cost = 0.0;
  
  int mode = ref(PandaDim::REFERENCE_DIMENSION - 1);
  //ROS_INFO_STREAM("mode: " << mode);
  robot_model_.update_state(x.head<BASE_ARM_GRIPPER_DIM>());
  //object_model_.update_state(x.segment<1>(2 * BASE_ARM_GRIPPER_DIM));
  cylinder_position_  <<  x(2*BASE_ARM_GRIPPER_DIM),
                          x(2*BASE_ARM_GRIPPER_DIM+1),
                          x(2*BASE_ARM_GRIPPER_DIM+2);

  double cylinder_radius = x(2*BASE_ARM_GRIPPER_DIM+OBJECT_DIMENSION+4);
  // ROS_INFO_STREAM("EE in world frame: " << robot_model_.get_pose(params_.tracked_frame).translation);

  // regularization cost
  cost += params_.Qreg *
          x.segment<BASE_ARM_GRIPPER_DIM>(BASE_ARM_GRIPPER_DIM).norm();
  
  // get robot EE pose
  EE_pose = robot_model_.get_pose(params_.tracked_frame);

  // end effector reaching cost
  if (mode == 0) {


    // for original EE pose tracking
    Eigen::Vector3d ref_t = ref.head<3>();
    Eigen::Quaterniond ref_q(ref.segment<4>(3));

    //ROS_INFO_STREAM( "ref EE  pose: " << ref_t.transpose() );
    robot_model_.get_error(params_.tracked_frame, ref_q, ref_t, error_);
    //ROS_INFO_STREAM( "error is: " << error_.transpose() );
    cost +=
        (error_.head<3>().transpose() * error_.head<3>()).norm() * params_.Qt;
    cost +=
        (error_.tail<3>().transpose() * error_.tail<3>()).norm() * params_.Qr;

    if (x(2 * BASE_ARM_GRIPPER_DIM + 2 * OBJECT_DIMENSION) > 0) 
      cost += params_.Qc;
  }

  // handle reaching cost
  else if (mode == 1) {
    

    Eigen::Vector2d position_dist;
    position_dist = EE_pose.translation.head<2>() - cylinder_position_.head<2>(); 
    cost += abs((position_dist.norm() - cylinder_radius)) * params_.Qt;
    // cost += abs( EE_pose.translation(2) - cylinder_position_(2) ) *params_.Qt;

    // adjust ee pose ONLY the orientation, when manipulating cylinder
    Eigen::Vector3d ref_t = ref.head<3>();
    Eigen::Quaterniond ref_q(ref.segment<4>(3));
    robot_model_.get_error(params_.tracked_frame, ref_q, ref_t, error_);
    // cost +=
    //     (error_.tail<3>().transpose() * error_.tail<3>()).norm() * params_.Qr;
    cost += error_(2)*error_(2)* params_.Qt2;

    // contact cost
    if (x(2*BASE_ARM_GRIPPER_DIM + 2*OBJECT_DIMENSION) > 0) {  //check if this is contanct term
      cost += params_.Qc;
    }
  }

  // object displacement cost
  else if (mode == 2) {

    Eigen::Vector3d ref_t = ref.head<3>();
    Eigen::Quaterniond ref_q(ref.segment<4>(3));
    Eigen::Vector3d pose_diff_1;
    double pose_diff, pose_error;
  
    pose_diff_1(0) = EE_pose.translation(0) - cylinder_position_(0);
    pose_diff_1(1) = EE_pose.translation(1) - cylinder_position_(1);
    pose_diff = pose_diff_1(0)*pose_diff_1(0) + pose_diff_1(1)*pose_diff_1(1);
    pose_diff_1(2) = abs(EE_pose.translation(2) - ref_t(2));
    robot_model_.get_error(params_.tracked_frame, ref_q, ref_t, error_);

    // EE with desired orientation
    cost += error_.tail<3>().norm() * params_.Qr ;

    // EE stay near the cylinder and on desired height
    pose_error = (pose_diff>params_.cylinder_radius*params_.cylinder_radius) ? sqrt(pose_diff)*1.5 : 0;
    cost += (pose_error*pose_error + pose_diff_1(2) ) *params_.Qt2;
    
    // object diff 
    Eigen::Vector2d object_position_diff;
    object_position_diff(0) = x(2 * BASE_ARM_GRIPPER_DIM) -
        ref(REFERENCE_POSE_DIMENSION + REFERENCE_OBSTACLE);
    object_position_diff(1) = x(2 * BASE_ARM_GRIPPER_DIM+1) -
        ref(REFERENCE_POSE_DIMENSION + REFERENCE_OBSTACLE+1);
    double object_error = object_position_diff.norm();
    cost += log2(abs(object_error+1)) * params_.Q_obj;
    
  }
  
  // // power cost
  cost += params_.Q_power * std::max(0.0, (-x.tail<12>().head<10>().transpose() * u.head<10>())(0) - params_.max_power); 

  // // self collision cost
  robot_model_.get_offset(params_.collision_link_0, params_.collision_link_1,
                          collision_vector_);
  cost += params_.Q_collision * std::pow(std::max(0.0, params_.collision_threshold - collision_vector_.norm()), 2);

  // // arm reach cost
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
  for (size_t i = 0; i < 8; i++) {
    if (x(i) < params_.lower_joint_limits[i])
      cost += params_.Q_joint_limit +
              params_.Q_joint_limit_slope *
                  std::pow(params_.lower_joint_limits[i] - x(i), 2);

    if (x(i) > params_.upper_joint_limits[i])
      cost += params_.Q_joint_limit +
              params_.Q_joint_limit_slope *
                  std::pow(x(i) - params_.upper_joint_limits[i], 2);
  }

  //ROS_INFO_STREAM(" overall cost is: " << cost);
  return cost;

}
