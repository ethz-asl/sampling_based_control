/*!
 * @file     controller_ros.cpp
 * @author   Giuseppe Rizzi
 * @date     03.09.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_manipulation/controller_interface.h"
#include <ros/package.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

using namespace manipulation;

bool PandaControllerInterface::init_ros() {
  optimal_trajectory_publisher_ = nh_.advertise<nav_msgs::Path>("/optimal_trajectory", 10);
  optimal_base_trajectory_publisher_ = nh_.advertise<nav_msgs::Path>("/optimal_base_trajectory", 10);
  obstacle_marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("/obstacle_marker", 10);
  base_twist_from_path_publisher_ = nh_.advertise<geometry_msgs::TwistStamped>("/twist_from_path", 10);
  pose_handle_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/handle_from_model", 10);
  
  mode_subscriber_ = nh_.subscribe("/mode", 10, &PandaControllerInterface::mode_callback, this);
  obstacle_subscriber_ =
      nh_.subscribe("/obstacle", 10, &PandaControllerInterface::obstacle_callback, this);
  ee_pose_desired_subscriber_ = nh_.subscribe(
      "/end_effector_pose_desired", 10, &PandaControllerInterface::ee_pose_desired_callback, this);

  if (!nh_.param<double>("obstacle_radius", obstacle_radius_, 0.0)) {
    ROS_ERROR("Could not parse obstacle_radius. Is the parameter set?");
    return false;
  }

  // initialize obstacle
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  try{
    transformStamped = tfBuffer.lookupTransform("world", "obstacle",
                             ros::Time(0), ros::Duration(3.0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    return false;
  }

  obstacle_marker_.header.frame_id = "world";
  obstacle_marker_.type = visualization_msgs::Marker::CYLINDER;
  obstacle_marker_.color.r = 1.0;
  obstacle_marker_.color.g = 0.0;
  obstacle_marker_.color.b = 0.0;
  obstacle_marker_.color.a = 0.4;
  obstacle_marker_.scale.x = 2.0 * obstacle_radius_;
  obstacle_marker_.scale.y = 2.0 * obstacle_radius_;
  obstacle_marker_.scale.z = 0.01;
  obstacle_marker_.pose.orientation.x = transformStamped.transform.rotation.x;
  obstacle_marker_.pose.orientation.y = transformStamped.transform.rotation.y;
  obstacle_marker_.pose.orientation.z = transformStamped.transform.rotation.z;
  obstacle_marker_.pose.orientation.w = transformStamped.transform.rotation.w;
  obstacle_marker_.pose.position.x = transformStamped.transform.translation.x;
  obstacle_marker_.pose.position.y = transformStamped.transform.translation.y;
  obstacle_marker_.pose.position.z = transformStamped.transform.translation.z;


  last_ee_ref_id_ = 0;
  ee_desired_pose_.header.seq = last_ee_ref_id_;

  last_ob_ref_id_ = 0;
  obstacle_pose_.header.seq = last_ob_ref_id_;

  optimal_path_.header.frame_id = "world";
  optimal_base_path_.header.frame_id = "world";
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
  std::string config_file;
  if (!nh_.param<std::string>("/config_file", config_file, "")) {
    throw std::runtime_error("Could not parse config_file. Is the parameter set?");
  }
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
  if (!nh_.param<bool>("fixed_base", fixed_base_, true)) {
    throw std::runtime_error("Could not parse fixed_base. Is the parameter set?");
  }
  ROS_INFO_STREAM("Fixed base? " << fixed_base_);

  PandaRaisimGains dyanmics_gains;
  if (!dyanmics_gains.parse_from_ros(nh_)){
    ROS_ERROR("Failed to parse dynamics gains.");
    return false;
  }
  ROS_INFO_STREAM("Successfully parsed dynamics gains: \n" << dyanmics_gains);
  dynamics = std::make_shared<PandaRaisimDynamics>(
      robot_description_raisim, object_description_raisim, config_.step_size, fixed_base_, dyanmics_gains);
  std::cout << "Done." << std::endl;

  // -------------------------------
  // cost
  // -------------------------------
  PandaCostParam cost_param;
  if (!cost_param.parse_from_ros(nh_)){
    ROS_ERROR("Failed to parse cost parameters.");
    return false;
  }
  ROS_INFO_STREAM("Successfully parsed cost params: \n" << cost_param);
  auto cost = std::make_shared<PandaCost>(robot_description, object_description, cost_param, fixed_base_);

  // -------------------------------
  // controller
  // -------------------------------
  controller = std::make_shared<mppi::PathIntegral>(dynamics, cost, config_);

  // -------------------------------
  // initialize reference
  // -------------------------------
  double object_reference_position;
  nh_.param<double>("object_reference_position", object_reference_position, 0.0);
  ref_.rr.resize(1, mppi::observation_t::Zero(PandaDim::REFERENCE_DIMENSION));
  ref_.rr[0](PandaDim::REFERENCE_POSE_DIMENSION + PandaDim::REFERENCE_OBSTACLE) =
      object_reference_position;
  
  // TODO(giuseppe) hack just to make sure obst not initialized on the way
  ref_.rr[0](7) = obstacle_marker_.pose.position.x;
  ref_.rr[0](8) = obstacle_marker_.pose.position.y;
  ref_.rr[0](9) = obstacle_marker_.pose.position.z;
  ref_.tt.resize(1, 0.0);

  ROS_INFO_STREAM("Reference initialized with: " << ref_.rr[0].transpose());
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
  get_controller()->set_reference_trajectory(ref_);
}

void PandaControllerInterface::obstacle_callback(const geometry_msgs::PoseStampedConstPtr& msg) {
  // std::unique_lock<std::mutex> lock(reference_mutex_);
  // obstacle_pose_ = *msg;
  // ref_.rr[0](7) = obstacle_pose_.pose.position.x;
  // ref_.rr[0](8) = obstacle_pose_.pose.position.y;
  // ref_.rr[0](9) = obstacle_pose_.pose.position.z;
}

void PandaControllerInterface::mode_callback(const std_msgs::Int64ConstPtr& msg) {
  std::unique_lock<std::mutex> lock(reference_mutex_);
  ref_.rr[0](11) = msg->data;
  get_controller()->set_reference_trajectory(ref_);
  ROS_INFO_STREAM("Switching to mode: " << msg->data);
}

bool PandaControllerInterface::update_reference() { return true; }

pinocchio::SE3 PandaControllerInterface::get_pose_end_effector(const Eigen::VectorXd& x) {
  if (fixed_base_) {
    pinocchio::forwardKinematics(model_, data_, x.head<ARM_GRIPPER_DIM>());
  } else {
    pinocchio::forwardKinematics(model_, data_, x.head<BASE_ARM_GRIPPER_DIM>());
  }
  pinocchio::updateFramePlacements(model_, data_);
  return data_.oMf[model_.getFrameId("panda_grasp")];
}

pinocchio::SE3 PandaControllerInterface::get_pose_handle(const Eigen::VectorXd& x) {
  pinocchio::forwardKinematics(object_model_, object_data_,
                               x.tail<2 * OBJECT_DIMENSION + CONTACT_STATE>().head<1>());
  pinocchio::updateFramePlacements(object_model_, object_data_);
  return object_data_.oMf[handle_idx_];
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

geometry_msgs::PoseStamped PandaControllerInterface::get_pose_base(const mppi::observation_t& x){
  geometry_msgs::PoseStamped pose_ros;
  pose_ros.header.stamp = ros::Time::now();
  pose_ros.header.frame_id = "world";
  pose_ros.pose.position.x = x(0);
  pose_ros.pose.position.y = x(1);
  pose_ros.pose.position.z = 0.0;
  pose_ros.pose.orientation.x = 0.0;
  pose_ros.pose.orientation.y = 0.0;
  pose_ros.pose.orientation.z = std::sin(0.5 * x(2));
  pose_ros.pose.orientation.w = std::cos(0.5 * x(2));
  return pose_ros;
}

geometry_msgs::PoseStamped PandaControllerInterface::get_pose_end_effector_ros(
    const Eigen::VectorXd& x) {
  pinocchio::SE3 pose = get_pose_end_effector(x);
  return pose_pinocchio_to_ros(pose);
}

geometry_msgs::PoseStamped PandaControllerInterface::get_pose_handle_ros(const Eigen::VectorXd& x) {
  pinocchio::SE3 pose = get_pose_handle(x);
  return pose_pinocchio_to_ros(pose);
}

void PandaControllerInterface::publish_ros() {
  obstacle_marker_publisher_.publish(obstacle_marker_);
  
  optimal_path_.header.stamp = ros::Time::now();
  optimal_path_.poses.clear();

  optimal_base_path_.header.stamp = ros::Time::now();
  optimal_base_path_.poses.clear();

  pinocchio::SE3 pose_temp;
  get_controller()->get_optimal_rollout(x_opt_, u_opt_);

  for (const auto& x : x_opt_) {
    optimal_path_.poses.push_back(pose_pinocchio_to_ros(get_pose_end_effector(x)));
    if (!fixed_base_)
      optimal_base_path_.poses.push_back(get_pose_base(x));
  }

  optimal_trajectory_publisher_.publish(optimal_path_);

  if (!fixed_base_) optimal_base_trajectory_publisher_.publish(optimal_base_path_);

  // extrapolate base twist from optimal base path
  if (optimal_base_path_.poses.size() > 2){
    geometry_msgs::TwistStamped base_twist_from_path;
    base_twist_from_path.header.frame_id = "world";
    base_twist_from_path.twist.linear.x = (optimal_base_path_.poses[1].pose.position.x - optimal_base_path_.poses[0].pose.position.x) / config_.step_size;
    base_twist_from_path.twist.linear.y = (optimal_base_path_.poses[1].pose.position.y - optimal_base_path_.poses[0].pose.position.y) / config_.step_size;
    base_twist_from_path.twist.linear.z = 0.0;

    base_twist_from_path.twist.angular.x = 0.0;
    base_twist_from_path.twist.angular.y = 0.0;
    base_twist_from_path.twist.angular.z = (x_opt_[1](3) - x_opt_[0](3)) / config_.step_size;
    base_twist_from_path_publisher_.publish(base_twist_from_path);
  }

  // for debug
  pose_handle_publisher_.publish(get_pose_handle_ros(x_opt_[0]));
}