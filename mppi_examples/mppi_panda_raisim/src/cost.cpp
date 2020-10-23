/*!
 * @file     pendulum_cart_cost.cpp
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_panda_raisim/cost.h"
#include "mppi_panda_raisim/dimensions.h"
#include <ros/package.h>

#define LOWER_LIMITS -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973
#define UPPER_LIMITS 2.8973, 1.7628, 2.8973, 0.0698, 2.8973, 3.7525, 2.8973

using namespace panda;

PandaCost::PandaCost(const std::string& robot_description, double linear_weight, double angular_weight, double obstacle_radius){

  robot_description_ = robot_description;
  linear_weight_ = linear_weight;
  angular_weight_ = angular_weight;
  obstacle_radius_ = obstacle_radius;

  pinocchio::urdf::buildModelFromXML(robot_description, model_);
  data_ = pinocchio::Data(model_);
  frame_id_ = model_.getFrameId(tracked_frame_);
  Q_linear_ = Eigen::Matrix3d::Identity() * linear_weight;
  Q_angular_ = Eigen::Matrix3d::Identity() * angular_weight;

  // TODO(giuseppe) remove hard coded joint limits
  joint_limits_lower_ << LOWER_LIMITS;
  joint_limits_upper_ << UPPER_LIMITS;
  std::cout << "Lower joints limits: " << joint_limits_lower_.transpose() << std::endl;
  std::cout << "Upper joints limits: " << joint_limits_upper_.transpose() << std::endl;

  // door model
  std::string data_dir = ros::package::getPath("mppi_panda_raisim") + "/data/";
  std::string door_urdf = data_dir + "door.urdf";
  pinocchio::urdf::buildModel(door_urdf, door_model_);
  door_data_ = pinocchio::Data(door_model_);
  handle_idx_ = door_model_.getFrameId("handle_link");

  Eigen::Quaterniond q(Eigen::AngleAxisd(-M_PI/2.0, Eigen::Vector3d::UnitY()));
  Eigen::Vector3d t;
  t.setZero();
  grasp_offset_ = pinocchio::SE3(q, t);
}

mppi::CostBase::cost_t PandaCost::compute_cost(const mppi::observation_t& x,
    const mppi::reference_t& ref, const double t) {

  static double linear_cost = 0;
  static double angular_cost = 0;
  double door_opening_cost = 0;
  double obstacle_cost = 0;
  double joint_limit_cost = 0;

  pose_current_ = get_pose_end_effector(x);

  // end effector reaching
  if (ref(PandaDim::REFERENCE_DIMENSION-1) == 0){
    Eigen::Vector3d ref_t = ref.head<3>();
    Eigen::Quaterniond ref_q(ref.segment<4>(3));
    pose_reference_ = pinocchio::SE3(ref_q, ref_t);
    err_ = pinocchio::log6(pose_current_.actInv(pose_reference_));
    linear_cost = err_.linear().transpose() * Q_linear_ * err_.linear();
    angular_cost = err_.angular().transpose() * Q_angular_ * err_.angular();
  }
  else{
    pose_handle_ = get_pose_handle(x);
    err_ = pinocchio::log6(pose_current_.actInv(pose_handle_.act(grasp_offset_)));
    linear_cost = err_.linear().transpose() * Q_linear_ * err_.linear();
    angular_cost = err_.angular().transpose() * Q_angular_ * err_.angular();

    // TODO(giuseppe) read the reference value from ref_
    door_opening_cost = std::pow(x(2*PandaDim::JOINT_DIMENSION)-M_PI/2, 2) * 10;
  }

  double obstacle_dist = (pose_current_.translation() - ref.segment<3>(7)).norm();
  if (obstacle_dist < obstacle_radius_)
    obstacle_cost =  Q_obst_;

  // joint limits
  /**
  for(size_t i=0; i<7; i++){

    if (x(i) < joint_limits_lower_(i))
      joint_limit_cost += 1000 + 100 * std::pow(joint_limits_lower_(i) - x(i), 2);

    if (x(i) > joint_limits_upper_(i))
      joint_limit_cost += 1000 + 100 * std::pow(x(i) - joint_limits_upper_(i), 2);
  }
   **/
  return linear_cost + angular_cost + obstacle_cost + joint_limit_cost + door_opening_cost;

}


pinocchio::SE3 PandaCost::get_pose_end_effector(const Eigen::VectorXd& x){
  pinocchio::forwardKinematics(model_, data_, x.head<PandaDim::JOINT_DIMENSION>());
  pinocchio::updateFramePlacements(model_, data_);
  return data_.oMf[frame_id_];
}

pinocchio::SE3 PandaCost::get_pose_handle(const Eigen::VectorXd& x){
  pinocchio::forwardKinematics(door_model_, door_data_, x.segment<1>(2*PandaDim::JOINT_DIMENSION));
  pinocchio::updateFramePlacements(door_model_, door_data_);
  return door_data_.oMf[handle_idx_];
}
