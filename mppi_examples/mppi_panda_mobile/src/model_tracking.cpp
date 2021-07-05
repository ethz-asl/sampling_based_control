//
// Created by giuseppe on 25.04.21.
//

#include "mppi_panda_mobile/model_tracking.h"

#include "mppi_panda_mobile/controller_interface.h"
#include "mppi_panda_mobile/cost.h"
#include "mppi_panda_mobile/dynamics.h"

#include <mppi/policies/gaussian_policy.h>
#include <mppi/policies/spline_policy.h>
#include <mppi_pinocchio/ros_conversions.h>
#include <mppi_ros/ros_params.h>
#include <ros/package.h>

using namespace panda_mobile;

PandaMobileModelTracking::PandaMobileModelTracking(ros::NodeHandle& nh)
    : nh_(nh) {
  initialized_ = true;
  initialized_ &= init_ros();
  initialized_ &= setup();
}

bool PandaMobileModelTracking::init_ros() {
  optimal_trajectory_publisher_ =
      nh_.advertise<nav_msgs::Path>("/optimal_trajectory", 10);
  obstacle_marker_publisher_ =
      nh_.advertise<visualization_msgs::Marker>("/obstacle_marker", 10);

  obstacle_subscriber_ = nh_.subscribe(
      "/obstacle", 10, &PandaMobileModelTracking::obstacle_callback, this);
  ee_pose_desired_subscriber_ =
      nh_.subscribe("/end_effector_pose_desired", 10,
                    &PandaMobileModelTracking::ee_pose_desired_callback, this);

  state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
  joint_state_.name = {"panda_joint1", "panda_joint2", "panda_joint3",
                       "panda_joint4", "panda_joint5", "panda_joint6",
                       "panda_joint7"};
  joint_state_.position.resize(7);
  joint_state_.header.frame_id = "base";

  // base tf
  world_base_tf_.header.frame_id = "world";
  world_base_tf_.child_frame_id = "base";

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

  optimal_path_.header.frame_id = "world";
  return true;
}

bool PandaMobileModelTracking::setup() {
  // Params
  std::string robot_description;
  double linear_weight;
  double angular_weight;
  bool holonomic;
  bool joint_limits;
  bool gaussian_policy;

  bool ok = true;
  ok &= mppi_ros::getString(nh_, "/robot_description", robot_description);
  ok &= mppi_ros::getNonNegative(nh_, "obstacle_radius", obstacle_radius_);
  ok &= mppi_ros::getNonNegative(nh_, "linear_weight", linear_weight);
  ok &= mppi_ros::getNonNegative(nh_, "angular_weight", angular_weight);
  ok &= mppi_ros::getBool(nh_, "holonomic", holonomic);
  ok &= mppi_ros::getBool(nh_, "joint_limits", joint_limits);
  ok &= mppi_ros::getBool(nh_, "gaussian_policy", gaussian_policy);
  if (!ok) {
    ROS_ERROR("Failed to parse parameters and set controller.");
    return false;
  }

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
  // internal model
  // -------------------------------
  robot_model_.init_from_xml(robot_description);

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
  // policy
  // -------------------------------
  std::shared_ptr<mppi::Policy> policy;
  if (gaussian_policy) {
    policy = std::make_shared<mppi::GaussianPolicy>(
        int(PandaMobileDim::INPUT_DIMENSION), config_);
  } else {
    policy = std::make_shared<mppi::SplinePolicy>(
        int(PandaMobileDim::INPUT_DIMENSION), config_);
  }

  // -------------------------------
  // initialize state
  // -------------------------------
  mppi::observation_t x0 =
      Eigen::VectorXd::Zero(PandaMobileDim::STATE_DIMENSION);
  auto x0v = nh_.param<std::vector<double>>("initial_configuration", {});
  for (int i = 0; i < x0v.size(); i++) x0(i) = x0v[i];

  // -------------------------------
  // controller TODO(giuseppe) change this to the proper way
  // -------------------------------
  init(dynamics, cost, policy, x0, 0.0, config_);

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

void PandaMobileModelTracking::ee_pose_desired_callback(
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
  set_reference(ref_);
}

void PandaMobileModelTracking::obstacle_callback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::unique_lock<std::mutex> lock(reference_mutex_);
  obstacle_pose_ = *msg;
  ref_.rr[0](7) = obstacle_pose_.pose.position.x;
  ref_.rr[0](8) = obstacle_pose_.pose.position.y;
  ref_.rr[0](9) = obstacle_pose_.pose.position.z;
  set_reference(ref_);
}

mppi_pinocchio::Pose PandaMobileModelTracking::get_pose_end_effector(
    const Eigen::VectorXd& x) {
  robot_model_.update_state(x);
  return robot_model_.get_pose("panda_hand");
}

geometry_msgs::PoseStamped PandaMobileModelTracking::get_pose_end_effector_ros(
    const Eigen::VectorXd& x) {
  mppi_pinocchio::Pose pose = get_pose_end_effector(x);
  geometry_msgs::PoseStamped pose_ros;
  pose_ros.header.stamp = ros::Time::now();
  pose_ros.header.frame_id = "world";
  mppi_pinocchio::to_msg(pose, pose_ros.pose);
  return pose_ros;
}

void PandaMobileModelTracking::publish_ros() {
  // Publish obstacle marker
  if (obstacle_pose_.header.seq != 0) {
    obstacle_marker_.pose.position = obstacle_pose_.pose.position;
    obstacle_marker_publisher_.publish(obstacle_marker_);
  }

  // Publish optimal rollout
  optimal_path_.header.stamp = ros::Time::now();
  optimal_path_.poses.clear();
  mppi_pinocchio::Pose pose_temp;
  geometry_msgs::PoseStamped pose_temp_ros;
  solver_->get_optimal_rollout(x_opt_, u_opt_);
  for (const auto& x : x_opt_) {
    pose_temp = get_pose_end_effector(x);
    mppi_pinocchio::to_msg(pose_temp, pose_temp_ros.pose);
    optimal_path_.poses.push_back(pose_temp_ros);
  }
  optimal_trajectory_publisher_.publish(optimal_path_);

  // publish joint state
  for (size_t i = 0; i < 7; i++) joint_state_.position[i] = x_(i + 3);
  joint_state_.header.stamp = ros::Time::now();
  state_publisher_.publish(joint_state_);

  // publish base transform
  world_base_tf_.header.stamp = ros::Time::now();
  world_base_tf_.transform.translation.x = x_(0);
  world_base_tf_.transform.translation.y = x_(1);
  Eigen::Quaterniond q(Eigen::AngleAxisd(x_(2), Eigen::Vector3d::UnitZ()));
  world_base_tf_.transform.rotation.x = q.x();
  world_base_tf_.transform.rotation.y = q.y();
  world_base_tf_.transform.rotation.z = q.z();
  world_base_tf_.transform.rotation.w = q.w();
  tf_broadcaster_.sendTransform(world_base_tf_);
}