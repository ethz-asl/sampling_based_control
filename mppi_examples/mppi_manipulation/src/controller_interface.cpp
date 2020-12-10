/*!
 * @file     controller_ros.cpp
 * @author   Giuseppe Rizzi
 * @date     03.09.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_manipulation/controller_interface.h"
#include <ros/package.h>

using namespace manipulation;

bool PandaControllerInterface::init_ros() {
  optimal_trajectory_publisher_ = nh_.advertise<nav_msgs::Path>("/optimal_trajectory", 10);
  obstacle_marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("/obstacle_marker", 10);

  mode_subscriber_ = nh_.subscribe("/mode", 10, &PandaControllerInterface::mode_callback, this);
  obstacle_subscriber_ =
      nh_.subscribe("/obstacle", 10, &PandaControllerInterface::obstacle_callback, this);
  ee_pose_desired_subscriber_ = nh_.subscribe(
      "/end_effector_pose_desired", 10, &PandaControllerInterface::ee_pose_desired_callback, this);

  obstacle_radius_ = param_io::param(nh_, "obstacle_radius", 0.2);
  obstacle_marker_.header.frame_id = "world";
  obstacle_marker_.type = visualization_msgs::Marker::SPHERE;
  obstacle_marker_.color.r = 1.0;
  obstacle_marker_.color.g = 0.0;
  obstacle_marker_.color.b = 0.0;
  obstacle_marker_.color.a = 0.4;
  obstacle_marker_.scale.x = 2.0 * obstacle_radius_;
  obstacle_marker_.scale.y = 2.0 * obstacle_radius_;
  obstacle_marker_.scale.z = 2.0 * obstacle_radius_;
  obstacle_marker_.pose.orientation.x = 0.0;
  obstacle_marker_.pose.orientation.y = 0.0;
  obstacle_marker_.pose.orientation.z = 0.0;
  obstacle_marker_.pose.orientation.w = 1.0;

  last_ee_ref_id_ = 0;
  ee_desired_pose_.header.seq = last_ee_ref_id_;

  last_ob_ref_id_ = 0;
  obstacle_pose_.header.seq = last_ob_ref_id_;

  optimal_path_.header.frame_id = "world";
  return true;
}

void PandaControllerInterface::init_model(const std::string& robot_description,
                                          const std::string& object_description) {
  pinocchio::urdf::buildModelFromXML(robot_description, model_);
  data_ = pinocchio::Data(model_);

  pinocchio::urdf::buildModelFromXML(object_description, object_model_);
  object_data_ = pinocchio::Data(object_model_);
  handle_idx_ = object_model_.getFrameId("handle_link");
}

bool PandaControllerInterface::set_controller(std::shared_ptr<mppi::PathIntegral>& controller) {
  // -------------------------------
  // internal model
  // -------------------------------
  std::string robot_description, object_description;
  if (!nh_.param<std::string>("/robot_description", robot_description, "")) {
    throw std::runtime_error("Could not parse robot description. Is the parameter set?");
  }
  if (!nh_.param<std::string>("/object_description", object_description, "")) {
    throw std::runtime_error("Could not parse object description. Is the parameter set?");
  }
  init_model(robot_description, object_description);

  // -------------------------------
  // config
  // -------------------------------
  std::string config_file = ros::package::getPath("mppi_manipulation") + "/params/params.yaml";
  if (!config_.init_from_file(config_file)) {
    ROS_ERROR_STREAM("Failed to init solver options from " << config_file);
    return false;
  }

  // -------------------------------
  // dynamics
  // -------------------------------
  mppi::DynamicsBase::dynamics_ptr dynamics;
  std::string robot_description_raisim, object_description_raisim;
  if (!nh_.param<std::string>("/robot_description_raisim", robot_description_raisim, "")) {
    throw std::runtime_error("Could not parse robot_description_raisim. Is the parameter set?");
  }
  if (!nh_.param<std::string>("/object_description_raisim", object_description_raisim, "")) {
    throw std::runtime_error("Could not parse object_description_raisim. Is the parameter set?");
  }
  dynamics = std::make_shared<PandaRaisimDynamics>(robot_description_raisim, object_description_raisim,
                                                   config_.step_size);

  // -------------------------------
  // cost
  // -------------------------------
  PandaCostParam cost_param;
  cost_param.Qo = 10000;
  cost_param.Qt = param_io::param(nh_, "linear_weight", 10.0);
  cost_param.Qr = param_io::param(nh_, "angular_weight", 10.0);
  cost_param.Qc = param_io::param(nh_, "contact_weight", 0.0);
  cost_param.ro = obstacle_radius_;

  using vd = std::vector<double>;
  vd grasp_t = param_io::param<vd>(nh_, "grasp_translation_offset", {});
  vd grasp_r = param_io::param<vd>(nh_, "grasp_orientation_offset", {});
  Eigen::Vector3d t(grasp_t[0], grasp_t[1], grasp_t[2]);
  Eigen::Quaterniond q(grasp_r[3], grasp_r[0], grasp_r[1], grasp_r[2]);
  pinocchio::SE3 grasp_offset(q, t);
  cost_param.grasp_offset = grasp_offset;

  auto cost = std::make_shared<PandaCost>(robot_description, object_description, cost_param);

  // -------------------------------
  // controller
  // -------------------------------
  controller = std::make_shared<mppi::PathIntegral>(dynamics, cost, config_);

  // -------------------------------
  // initialize reference
  // -------------------------------
  ref_.rr.resize(1, mppi::observation_t::Zero(PandaDim::REFERENCE_DIMENSION));
  ref_.tt.resize(1, 0.0);
  return true;
}

void PandaControllerInterface::ee_pose_desired_callback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::unique_lock<std::mutex> lock(reference_mutex_);
  ee_desired_pose_ = *msg;
  Eigen::VectorXd pr = Eigen::VectorXd::Zero(7);
  ref_.rr[0].head<7>()(0) = msg->pose.position.x;
  ref_.rr[0].head<7>()(1) = msg->pose.position.y;
  ref_.rr[0].head<7>()(2) = msg->pose.position.z;
  ref_.rr[0].head<7>()(3) = msg->pose.orientation.x;
  ref_.rr[0].head<7>()(4) = msg->pose.orientation.y;
  ref_.rr[0].head<7>()(5) = msg->pose.orientation.z;
  ref_.rr[0].head<7>()(6) = msg->pose.orientation.w;
}

void PandaControllerInterface::obstacle_callback(const geometry_msgs::PoseStampedConstPtr& msg) {
  std::unique_lock<std::mutex> lock(reference_mutex_);
  obstacle_pose_ = *msg;
  ref_.rr[0](7) = obstacle_pose_.pose.position.x;
  ref_.rr[0](8) = obstacle_pose_.pose.position.y;
  ref_.rr[0](9) = obstacle_pose_.pose.position.z;
}

void PandaControllerInterface::mode_callback(const std_msgs::Int64ConstPtr& msg) {
  std::unique_lock<std::mutex> lock(reference_mutex_);
  ref_.rr[0](11) = msg->data;
  ROS_INFO_STREAM("Switching to mode: " << msg->data);
}

bool PandaControllerInterface::update_reference() {
  std::unique_lock<std::mutex> lock(reference_mutex_);
  if (last_ee_ref_id_ != ee_desired_pose_.header.seq ||
      (last_ob_ref_id_ != obstacle_pose_.header.seq && ee_desired_pose_.header.seq != 0)) {
    get_controller()->set_reference_trajectory(ref_);
  }
  last_ee_ref_id_ = ee_desired_pose_.header.seq;
  last_ob_ref_id_ = obstacle_pose_.header.seq;
  return true;
}

pinocchio::SE3 PandaControllerInterface::get_pose_end_effector(const Eigen::VectorXd& x) {
  pinocchio::forwardKinematics(model_, data_, x.head<PandaDim::JOINT_DIMENSION>());
  pinocchio::updateFramePlacements(model_, data_);
  return data_.oMf[model_.getFrameId("panda_grasp")];
}

geometry_msgs::PoseStamped PandaControllerInterface::pose_pinocchio_to_ros(
    const pinocchio::SE3& pose) {
  geometry_msgs::PoseStamped pose_ros;
  pose_ros.header.stamp = ros::Time::now();
  pose_ros.header.frame_id = "world";
  pose_ros.pose.position.x = pose.translation()(0);
  pose_ros.pose.position.y = pose.translation()(1);
  pose_ros.pose.position.z = pose.translation()(2);
  Eigen::Quaterniond q(pose.rotation());
  pose_ros.pose.orientation.x = q.x();
  pose_ros.pose.orientation.y = q.y();
  pose_ros.pose.orientation.z = q.z();
  pose_ros.pose.orientation.w = q.w();
  return pose_ros;
}

geometry_msgs::PoseStamped PandaControllerInterface::get_pose_end_effector_ros(
    const Eigen::VectorXd& x) {
  pinocchio::SE3 pose = get_pose_end_effector(x);
  return pose_pinocchio_to_ros(pose);
}

pinocchio::SE3 PandaControllerInterface::get_pose_handle(const Eigen::VectorXd& x) {
  pinocchio::forwardKinematics(object_model_, object_data_,
                               x.segment<1>(2 * PandaDim::JOINT_DIMENSION));
  pinocchio::updateFramePlacements(object_model_, object_data_);
  return object_data_.oMf[handle_idx_];
}

geometry_msgs::PoseStamped PandaControllerInterface::get_pose_handle_ros(const Eigen::VectorXd& x) {
  pinocchio::SE3 pose = get_pose_handle(x);
  return pose_pinocchio_to_ros(pose);
}

void PandaControllerInterface::publish_ros() {
  if (obstacle_pose_.header.seq != 0) {  // obstacle set at least once
    obstacle_marker_.pose.position = obstacle_pose_.pose.position;
    obstacle_marker_publisher_.publish(obstacle_marker_);
  }

  optimal_path_.header.stamp = ros::Time::now();
  optimal_path_.poses.clear();
  pinocchio::SE3 pose_temp;
  get_controller()->get_optimal_rollout(x_opt_, u_opt_);

  for (const auto& x : x_opt_) {
    optimal_path_.poses.push_back(pose_pinocchio_to_ros(get_pose_end_effector(x)));
  }
  optimal_trajectory_publisher_.publish(optimal_path_);
}