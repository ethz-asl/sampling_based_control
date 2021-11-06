/*!
* @file     controller_interface_no_ros.cpp
* @author   Giulio Schiavi
* @date     04.11.21
* @version  1.0
* @brief    description
*/
#include "mppi/core/config.h"
#include "mppi_manipulation/controller_interface_no_ros.h"
#include <mppi_pinocchio/ros_conversions.h>
#include "mppi_manipulation/cost.h"
#include "mppi_manipulation/dynamics.h"
#include "mppi_manipulation/params/cost_params.h"
#include "mppi_manipulation/params/dynamics_params.h"

#include <mppi/policies/gaussian_policy.h>
#include <mppi/policies/spline_policy.h>

using namespace manipulation;

PandaControllerInterfaceNoRos::PandaControllerInterfaceNoRos(const std::string& config_path){
  manipulation_config_path_ = config_path;
  if(manipulation_config_.init_from_file(config_path)){
    ROS_INFO_STREAM("Successfully loaded config file at " << config_path);
  }else{
    ROS_ERROR("Failed to parse manipulation config");
  }
}
bool PandaControllerInterfaceNoRos::init_ros(){
  default_pose_ = manipulation_config_.default_pose;
  object_tolerance_ = manipulation_config_.object_tolerance;
  reference_scheduler_.parse_from_file(manipulation_config_.references_file);
  last_ee_ref_id_ = 0;
  ee_desired_pose_.header.seq = last_ee_ref_id_;

  last_ob_ref_id_ = 0;
  obstacle_pose_.header.seq = last_ob_ref_id_;

  optimal_path_.header.frame_id = "world";
  optimal_base_path_.header.frame_id = "world";
  reference_set_ = false;
  ROS_INFO("[PandaControllerInterface::init_ros] ok!");
  return true;
}

bool PandaControllerInterfaceNoRos::init() {
  init_ros();

  bool ok;
  try {
    ok = set_controller(controller_);
  } catch (std::runtime_error &err) {
    ROS_ERROR_STREAM(err.what());
    ok = false;
  } catch (...) {
    ROS_ERROR("Unknown exception caught while setting the controller.");
    ok = false;
  }
  if (controller_ == nullptr || !ok) return false;
  ROS_INFO("Controller interface initialized.");
  return true;
}


void PandaControllerInterfaceNoRos::set_observation(const mppi::observation_t &x,
                                    const double &t) {
  controller_->set_observation(x, t);
  observation_set_ = true;
}

bool PandaControllerInterfaceNoRos::update_policy() {
  controller_->update_policy();
  return true;
}

mppi::input_t PandaControllerInterfaceNoRos::get_input(const mppi::observation_t &x, const double &t) {
  mppi::input_t u;
  controller_->get_input(x, u, t);
  return u;
}

void PandaControllerInterfaceNoRos::set_mode(int mode) {
  std::unique_lock<std::mutex> lock(reference_mutex_);
  ref_.rr[0](11) = mode;
  get_controller()->set_reference_trajectory(ref_);
  local_cost_->set_reference_trajectory(ref_);
  reference_set_ = true;
}

void PandaControllerInterfaceNoRos::set_desired_ee_pose(
    const std::vector<double> pose) {
  /*
   * pose is [position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w]
   */
  std::unique_lock<std::mutex> lock(reference_mutex_);
  Eigen::VectorXd pr = Eigen::VectorXd::Zero(7);
  ref_.rr[0].head<7>()(0) = pose[0];
  ref_.rr[0].head<7>()(1) = pose[1];
  ref_.rr[0].head<7>()(2) = pose[2];
  ref_.rr[0].head<7>()(3) = pose[3];
  ref_.rr[0].head<7>()(4) = pose[4];
  ref_.rr[0].head<7>()(5) = pose[5];
  ref_.rr[0].head<7>()(6) = pose[6];
  get_controller()->set_reference_trajectory(ref_);
  local_cost_->set_reference_trajectory(ref_);
  reference_set_ = true;
}

void PandaControllerInterfaceNoRos::update_reference(const mppi::observation_t& x,
                                                     const double t) {
  if (reference_scheduler_.has_reference(t)) {
    reference_scheduler_.set_reference(t, ref_);

    std::unique_lock<std::mutex> lock(reference_mutex_);
    get_controller()->set_reference_trajectory(ref_);
    local_cost_->set_reference_trajectory(ref_);
    reference_set_ = true;
  }
}

void PandaControllerInterfaceNoRos::init_model(
    const std::string& robot_description,
    const std::string& object_description) {
  robot_model_.init_from_xml(robot_description);
  object_model_.init_from_xml(object_description);
  ROS_INFO("[PandaControllerInterface::init_model] ok!");
}

bool PandaControllerInterfaceNoRos::set_controller(mppi::solver_ptr& controller) {
  // -------------------------------
  // internal model
  // -------------------------------
  init_model(manipulation_config_.robot_description, manipulation_config_.object_description);

  // -------------------------------
  // dynamics
  // -------------------------------
  mppi::dynamics_ptr dynamics;
  dynamics_params_.init_from_config(manipulation_config_);
  ROS_INFO_STREAM("Successfully parsed controller dynamics parameters: " << dynamics_params_);
  dynamics = std::make_shared<PandaRaisimDynamics>(dynamics_params_);

  // -------------------------------
  // config
  // -------------------------------
  if (!config_.init_from_file(manipulation_config_path_)) {
    ROS_ERROR_STREAM("Failed to init solver options from " << manipulation_config_.controller_config_file);
    return false;
  }

  // -------------------------------
  // cost
  // -------------------------------
  CostParams cost_params;
  if (!cost_params.init_from_config(manipulation_config_)) {
    ROS_ERROR("Failed to parse cost parameters.");
    return false;
  }

  auto cost = std::make_shared<PandaCost>(cost_params);
  local_cost_ = std::make_unique<manipulation::PandaCost>(cost_params);

  // -------------------------------
  // policy
  // -------------------------------
  std::shared_ptr<mppi::Policy> policy;
  if (manipulation_config_.gaussian_policy) {
    policy = std::make_shared<mppi::GaussianPolicy>(
        dynamics->get_input_dimension(), config_);
  } else {
    policy = std::make_shared<mppi::SplinePolicy>(
        dynamics->get_input_dimension(), config_);
  }

  // -------------------------------
  // controller
  // -------------------------------
  controller = std::make_shared<mppi::Solver>(dynamics, cost, policy, config_);

  // -------------------------------
  // initialize reference
  // -------------------------------
  ref_.rr.resize(1, mppi::observation_t::Zero(PandaDim::REFERENCE_DIMENSION));
  // init obstacle fare away
  ref_.rr[0](7) = 100;
  ref_.rr[0](8) = 100;
  ref_.rr[0](9) = 100;
  ref_.rr[0].tail<1>()(0) = 0.0;
  ref_.tt.resize(1, 0.0);
  return true;
}

bool PandaControllerInterfaceNoRos::init_reference_to_current_pose(
    const mppi::observation_t& x, const double t) {
  auto ee_pose = get_pose_end_effector(x);
  ref_.rr.resize(1, mppi::observation_t::Zero(PandaDim::REFERENCE_DIMENSION));
  ref_.rr[0].head<3>() = ee_pose.translation;
  ref_.rr[0].segment<4>(3) = ee_pose.rotation.coeffs();

  // init obstacle fare away
  ref_.rr[0](7) = 100;
  ref_.rr[0](8) = 100;
  ref_.rr[0](9) = 100;

  // mode
  ref_.rr[0].tail<1>()(0) = 0.0;

  // at time zero
  ref_.tt.resize(1, t);

  get_controller()->set_reference_trajectory(ref_);
  local_cost_->set_reference_trajectory(ref_);
  reference_set_ = true;
  ROS_INFO_STREAM("Initializing reference to the current pose: \n"
                  << ref_.rr[0].transpose());
  return true;
}

double PandaControllerInterfaceNoRos::get_stage_cost(const mppi::observation_t& x,
                                                const mppi::input_t& u,
                                                const double t) {
  if (!reference_set_) return -1.0;
  return local_cost_->get_stage_cost(x, u, t);
}

mppi_pinocchio::Pose PandaControllerInterfaceNoRos::get_pose_end_effector(
    const Eigen::VectorXd& x) {
    robot_model_.update_state(x.head<BASE_ARM_GRIPPER_DIM>());
  return robot_model_.get_pose("panda_grasp");
}

mppi_pinocchio::Pose PandaControllerInterfaceNoRos::get_pose_handle(
    const Eigen::VectorXd& x) {
  object_model_.update_state(
      x.tail<2 * OBJECT_DIMENSION + CONTACT_STATE>().head<1>());
  return object_model_.get_pose(dynamics_params_.object_handle_link);
}

//geometry_msgs::PoseStamped PandaControllerInterfaceNoRos::get_pose_base(
//    const mppi::observation_t& x) {
//  geometry_msgs::PoseStamped pose_ros;
//  pose_ros.header.stamp = ros::Time::now();
//  pose_ros.header.frame_id = "world";
//  pose_ros.pose.position.x = x(0);
//  pose_ros.pose.position.y = x(1);
//  pose_ros.pose.position.z = 0.0;
//  pose_ros.pose.orientation.x = 0.0;
//  pose_ros.pose.orientation.y = 0.0;
//  pose_ros.pose.orientation.z = std::sin(0.5 * x(2));
//  pose_ros.pose.orientation.w = std::cos(0.5 * x(2));
//  return pose_ros;
//}
//
//geometry_msgs::PoseStamped PandaControllerInterfaceNoRos::get_pose_end_effector_ros(
//    const Eigen::VectorXd& x) {
//  geometry_msgs::PoseStamped pose_ros;
//  pose_ros.header.frame_id = "world";
//  pose_ros.header.stamp = ros::Time::now();
//  mppi_pinocchio::Pose pose = get_pose_end_effector(x);
//  mppi_pinocchio::to_msg(pose, pose_ros.pose);
//  return pose_ros;
//}
//
//geometry_msgs::PoseStamped PandaControllerInterfaceNoRos::get_pose_handle_ros(
//    const Eigen::VectorXd& x) {
//  geometry_msgs::PoseStamped pose_ros;
//  pose_ros.header.frame_id = "world";
//  pose_ros.header.stamp = ros::Time::now();
//  mppi_pinocchio::Pose pose = get_pose_handle(x);
//  mppi_pinocchio::to_msg(pose, pose_ros.pose);
//  return pose_ros;
//}