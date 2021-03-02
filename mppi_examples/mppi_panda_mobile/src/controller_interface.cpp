/*!
 * @file     controller_interface.cpp
 * @author   Giuseppe Rizzi
 * @date     25.09.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_panda_mobile/controller_interface.h"
#include "mppi_panda_mobile/cost.h"
#include "mppi_panda_mobile/dynamics.h"

#include <mppi_pinocchio/ros_conversions.h>
#include <mppi_ros/ros_params.h>
#include <ros/package.h>

using namespace panda_mobile;

bool PandaMobileControllerInterface::init_ros() {
  optimal_trajectory_publisher_ =
      nh_.advertise<nav_msgs::Path>("/optimal_trajectory", 10);
  obstacle_marker_publisher_ =
      nh_.advertise<visualization_msgs::Marker>("/obstacle_marker", 10);

  obstacle_subscriber_ =
      nh_.subscribe("/obstacle", 10,
                    &PandaMobileControllerInterface::obstacle_callback, this);
  ee_pose_desired_subscriber_ = nh_.subscribe(
      "/end_effector_pose_desired", 10,
      &PandaMobileControllerInterface::ee_pose_desired_callback, this);

  if (!mppi_ros::getNonNegative(nh_, "obstacle_radius", obstacle_radius_))
    return false;

  obstacle_marker_.header.frame_id = "world";
  obstacle_marker_.type = visualization_msgs::Marker::SPHERE;
  obstacle_marker_.color.r = 1.0;
  obstacle_marker_.color.g = 0.0;
  obstacle_marker_.color.b = 0.0;
  obstacle_marker_.color.a = 0.4;
  obstacle_marker_.scale.x = 2.0 * 0.01;
  obstacle_marker_.scale.y = 2.0 * 0.01;
  obstacle_marker_.scale.z = 2.0 * 0.01;
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

void PandaMobileControllerInterface::init_model(
    const std::string& robot_description) {
  robot_model_.init_from_xml(robot_description);
}

bool PandaMobileControllerInterface::set_controller(
    std::shared_ptr<mppi::PathIntegral>& controller) {
  // Params
  std::string robot_description;
  double linear_weight;
  double angular_weight;
  bool holonomic;
  bool joint_limits;

  bool ok = true;
  ok &= mppi_ros::getString(nh_, "/robot_description", robot_description);
  ok &= mppi_ros::getNonNegative(nh_, "obstacle_radius", obstacle_radius_);
  ok &= mppi_ros::getNonNegative(nh_, "linear_weight", linear_weight);
  ok &= mppi_ros::getNonNegative(nh_, "angular_weight", angular_weight);
  ok &= mppi_ros::getBool(nh_, "holonomic", holonomic);
  ok &= mppi_ros::getBool(nh_, "joint_limits", joint_limits);
  if (!ok) {
    ROS_ERROR("Failed to parse parameters and set controller.");
    return false;
  }

  // -------------------------------
  // internal model
  // -------------------------------
  init_model(robot_description);

  // -------------------------------
  // dynamics
  // -------------------------------
  auto dynamics =
      std::make_shared<PandaMobileDynamics>(robot_description, holonomic);

  // -------------------------------
  // cost
  // -------------------------------
  auto cost = std::make_shared<PandaMobileCost>(robot_description,
                                                linear_weight, angular_weight,
                                                obstacle_radius_, joint_limits);

  // -------------------------------
  // config
  // -------------------------------
  std::string config_file =
      ros::package::getPath("mppi_panda_mobile") + "/config/params.yaml";
  if (!config_.init_from_file(config_file)) {
    ROS_ERROR_STREAM("Failed to init solver options from " << config_file);
    return false;
  }

  // -------------------------------
  // controller
  // -------------------------------
  controller = std::make_shared<mppi::PathIntegral>(dynamics, cost, config_);

  // -------------------------------
  // initialize reference
  // -------------------------------
  ref_.rr.resize(
      1, mppi::observation_t::Zero(PandaMobileDim::REFERENCE_DIMENSION));
  ref_.tt.resize(1, 0.0);

  // -------------------------------
  // obstacle marker
  // -------------------------------
  obstacle_marker_.scale.x = 2.0 * obstacle_radius_;
  obstacle_marker_.scale.y = 2.0 * obstacle_radius_;
  obstacle_marker_.scale.z = 2.0 * obstacle_radius_;
  return true;
}

void PandaMobileControllerInterface::ee_pose_desired_callback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::unique_lock<std::mutex> lock(reference_mutex_);
  ee_desired_pose_ = *msg;
  Eigen::VectorXd pr = Eigen::VectorXd::Zero(7);
  pr(0) = msg->pose.position.x;
  pr(1) = msg->pose.position.y;
  pr(2) = msg->pose.position.z;
  pr(3) = msg->pose.orientation.x;
  pr(4) = msg->pose.orientation.y;
  pr(5) = msg->pose.orientation.z;
  pr(6) = msg->pose.orientation.w;
  ref_.rr[0].head<7>() = pr;
}

void PandaMobileControllerInterface::obstacle_callback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::unique_lock<std::mutex> lock(reference_mutex_);
  obstacle_pose_ = *msg;
  ref_.rr[0](7) = obstacle_pose_.pose.position.x;
  ref_.rr[0](8) = obstacle_pose_.pose.position.y;
  ref_.rr[0](9) = obstacle_pose_.pose.position.z;
}

bool PandaMobileControllerInterface::update_reference() {
  std::unique_lock<std::mutex> lock(reference_mutex_);
  if (last_ee_ref_id_ != ee_desired_pose_.header.seq ||
      (last_ob_ref_id_ != obstacle_pose_.header.seq &&
       ee_desired_pose_.header.seq != 0)) {
    get_controller()->set_reference_trajectory(ref_);
  }
  last_ee_ref_id_ = ee_desired_pose_.header.seq;
  last_ob_ref_id_ = obstacle_pose_.header.seq;
  return true;
}

mppi_pinocchio::Pose PandaMobileControllerInterface::get_pose_end_effector(
    const Eigen::VectorXd& x) {
  robot_model_.update_state(x.head<7>());
  mppi_pinocchio::Pose base_pose;
  base_pose.translation = Eigen::Vector3d(x(7), x(8), 0.0);
  base_pose.rotation =
      Eigen::Quaterniond(Eigen::AngleAxisd(x(9), Eigen::Vector3d::UnitZ()));
  mppi_pinocchio::Pose arm_pose = robot_model_.get_pose("panda_hand");
  return base_pose * arm_pose;
}

geometry_msgs::PoseStamped
PandaMobileControllerInterface::get_pose_end_effector_ros(
    const Eigen::VectorXd& x) {
  mppi_pinocchio::Pose pose = get_pose_end_effector(x);
  geometry_msgs::PoseStamped pose_ros;
  pose_ros.header.stamp = ros::Time::now();
  pose_ros.header.frame_id = "world";
  mppi_pinocchio::to_msg(pose, pose_ros.pose);
  return pose_ros;
}

void PandaMobileControllerInterface::publish_ros() {
  if (obstacle_pose_.header.seq != 0) {  // obstacle set at least once
    obstacle_marker_.pose.position = obstacle_pose_.pose.position;
    obstacle_marker_publisher_.publish(obstacle_marker_);
  }

  optimal_path_.header.stamp = ros::Time::now();
  optimal_path_.poses.clear();
  mppi_pinocchio::Pose pose_temp;
  geometry_msgs::PoseStamped pose_temp_ros;
  get_controller()->get_optimal_rollout(x_opt_, u_opt_);

  for (const auto& x : x_opt_) {
    pose_temp = get_pose_end_effector(x);
    mppi_pinocchio::to_msg(pose_temp, pose_temp_ros.pose);
    optimal_path_.poses.push_back(pose_temp_ros);
  }
  optimal_trajectory_publisher_.publish(optimal_path_);
}