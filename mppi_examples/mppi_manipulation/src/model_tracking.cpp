//
// Created by giuseppe on 04.05.21.
//

#include "mppi_manipulation/model_tracking.h"
#include "mppi_manipulation/cost.h"
#include "mppi_manipulation/dynamics.h"

#include <mppi_pinocchio/ros_conversions.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/package.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

using namespace manipulation;

ManipulationTrackingController::ManipulationTrackingController(
    ros::NodeHandle& nh)
    : nh_(nh) {
  initialized_ = true;
  initialized_ &= init_ros();
  initialized_ &= setup();
}

bool ManipulationTrackingController::init_ros() {
  optimal_trajectory_publisher_ =
      nh_.advertise<nav_msgs::Path>("/optimal_trajectory", 10);
  optimal_base_trajectory_publisher_ =
      nh_.advertise<nav_msgs::Path>("/optimal_base_trajectory", 10);
  obstacle_marker_publisher_ =
      nh_.advertise<visualization_msgs::Marker>("/obstacle_marker", 10);
  base_twist_from_path_publisher_ =
      nh_.advertise<geometry_msgs::TwistStamped>("/twist_from_path", 10);
  pose_handle_publisher_ =
      nh_.advertise<geometry_msgs::PoseStamped>("/handle_from_model", 10);

  mode_subscriber_ = nh_.subscribe(
      "/mode", 10, &ManipulationTrackingController::mode_callback, this);
  ee_pose_desired_subscriber_ = nh_.subscribe(
      "/end_effector_pose_desired", 10,
      &ManipulationTrackingController::ee_pose_desired_callback, this);

  // initialize obstacle
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  try {
    transformStamped = tfBuffer.lookupTransform(
        "world", "obstacle", ros::Time(0), ros::Duration(3.0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  obstacle_marker_.header.frame_id = "world";
  obstacle_marker_.type = visualization_msgs::Marker::CYLINDER;
  obstacle_marker_.color.r = 1.0;
  obstacle_marker_.color.g = 0.0;
  obstacle_marker_.color.b = 0.0;
  obstacle_marker_.color.a = 0.4;
  obstacle_marker_.scale.x = 2.0 * 0.01;
  obstacle_marker_.scale.y = 2.0 * 0.01;
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

  state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
  object_state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("/object/joint_state", 10);
  contact_forces_publisher_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/contact_forces", 10);
  ee_publisher_ =
      nh_.advertise<geometry_msgs::PoseStamped>("/end_effector", 10);
  handle_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/handle", 10);

  joint_state_.header.frame_id = "world";
  object_state_.name = {"articulation_joint"};
  object_state_.position.resize(1);

  force_marker_.type = visualization_msgs::Marker::ARROW;
  force_marker_.header.frame_id = "world";
  force_marker_.action = visualization_msgs::Marker::ADD;
  force_marker_.pose.orientation.w = 1.0;
  force_marker_.scale.x = 0.005;
  force_marker_.scale.y = 0.01;
  force_marker_.scale.z = 0.02;
  force_marker_.color.r = 1.0;
  force_marker_.color.b = 0.0;
  force_marker_.color.g = 0.0;
  force_marker_.color.a = 1.0;

  return true;
}

void ManipulationTrackingController::init_model(
    const std::string& robot_description,
    const std::string& object_description) {
  robot_model_.init_from_xml(robot_description);
  object_model_.init_from_xml(object_description);
}

bool ManipulationTrackingController::setup() {
  // -------------------------------
  // internal model
  // -------------------------------
  std::string robot_description, object_description;
  if (!nh_.param<std::string>("/robot_description", robot_description, "")) {
    throw std::runtime_error(
        "Could not parse robot description. Is the parameter set?");
  }
  if (!nh_.param<std::string>("/object_description", object_description, "")) {
    throw std::runtime_error(
        "Could not parse object description. Is the parameter set?");
  }
  init_model(robot_description, object_description);

  // -------------------------------
  // config
  // -------------------------------
  std::string config_file;
  if (!nh_.param<std::string>("/config_file", config_file, "")) {
    throw std::runtime_error(
        "Could not parse config_file. Is the parameter set?");
  }
  if (!config_.init_from_file(config_file)) {
    ROS_ERROR_STREAM("Failed to init solver options from " << config_file);
    return false;
  }

  // -------------------------------
  // dynamics
  // -------------------------------
  mppi::dynamics_ptr dynamics;
  std::string robot_description_raisim, object_description_raisim;
  if (!nh_.param<std::string>("/robot_description_raisim",
                              robot_description_raisim, "")) {
    throw std::runtime_error(
        "Could not parse robot_description_raisim. Is the parameter set?");
  }
  if (!nh_.param<std::string>("/object_description_raisim",
                              object_description_raisim, "")) {
    throw std::runtime_error(
        "Could not parse object_description_raisim. Is the parameter set?");
  }
  if (!nh_.param<bool>("fixed_base", fixed_base_, true)) {
    throw std::runtime_error(
        "Could not parse fixed_base. Is the parameter set?");
  }

  ROS_INFO_STREAM("Fixed base? " << fixed_base_);
  if (fixed_base_) {
    joint_state_.name = {
        "panda_joint1", "panda_joint2",        "panda_joint3",
        "panda_joint4", "panda_joint5",        "panda_joint6",
        "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"};
  } else {
    joint_state_.name = {
        "x_base_joint", "y_base_joint",        "pivot_joint",
        "panda_joint1", "panda_joint2",        "panda_joint3",
        "panda_joint4", "panda_joint5",        "panda_joint6",
        "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"};
  }
  joint_state_.position.resize(joint_state_.name.size());
  joint_state_.velocity.resize(joint_state_.name.size());

  PandaRaisimGains dyanmics_gains;
  if (!dyanmics_gains.parse_from_ros(nh_)) {
    ROS_ERROR("Failed to parse dynamics gains.");
    return false;
  }
  ROS_INFO_STREAM("Successfully parsed dynamics gains: \n" << dyanmics_gains);
  dynamics = std::make_shared<PandaRaisimDynamics>(
      robot_description_raisim, object_description_raisim, config_.step_size,
      fixed_base_, dyanmics_gains);
  std::cout << "Done." << std::endl;

  // -------------------------------
  // cost
  // -------------------------------
  PandaCostParam cost_param;
  if (!cost_param.parse_from_ros(nh_)) {
    ROS_ERROR("Failed to parse cost parameters.");
    return false;
  }
  ROS_INFO_STREAM("Successfully parsed cost params: \n" << cost_param);
  auto cost = std::make_shared<PandaCost>(robot_description, object_description,
                                          cost_param, fixed_base_);

  // ----------------------------------
  // initial state
  // set initial state (which is also equal to the one to be tracked)
  // the object position and velocity is already set to 0
  observation_t x0 = observation_t::Zero(dynamics->get_state_dimension());
  auto x0v = nh_.param<std::vector<double>>("initial_configuration", {});
  for (size_t i = 0; i < x0v.size(); i++) x0(i) = x0v[i];

  // -------------------------------
  // controller TODO(giuseppe) change this to the proper way
  // -------------------------------
  init(dynamics, cost, x0, 0.0, config_);

  // -------------------------------
  // initialize reference
  // -------------------------------
  double object_reference_position;
  nh_.param<double>("object_reference_position", object_reference_position,
                    0.0);
  ref_.rr.resize(1, mppi::observation_t::Zero(PandaDim::REFERENCE_DIMENSION));
  ref_.rr[0](PandaDim::REFERENCE_POSE_DIMENSION +
             PandaDim::REFERENCE_OBSTACLE) = object_reference_position;

  // TODO(giuseppe) hack just to make sure obst not initialized on the way
  ref_.rr[0](7) = obstacle_marker_.pose.position.x;
  ref_.rr[0](8) = obstacle_marker_.pose.position.y;
  ref_.rr[0](9) = obstacle_marker_.pose.position.z;
  ref_.tt.resize(1, 0.0);

  ROS_INFO_STREAM("Reference initialized with: " << ref_.rr[0].transpose());
  return true;
}

void ManipulationTrackingController::ee_pose_desired_callback(
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
  set_reference(ref_);
}

void ManipulationTrackingController::mode_callback(
    const std_msgs::Int64ConstPtr& msg) {
  std::unique_lock<std::mutex> lock(reference_mutex_);
  ref_.rr[0](11) = msg->data;
  set_reference(ref_);
  ROS_INFO_STREAM("Switching to mode: " << msg->data);
}

mppi_pinocchio::Pose ManipulationTrackingController::get_pose_end_effector(
    const Eigen::VectorXd& x) {
  if (fixed_base_) {
    robot_model_.update_state(x.head<ARM_GRIPPER_DIM>());
  } else {
    robot_model_.update_state(x.head<BASE_ARM_GRIPPER_DIM>());
  }
  return robot_model_.get_pose("panda_grasp");
}

mppi_pinocchio::Pose ManipulationTrackingController::get_pose_handle(
    const Eigen::VectorXd& x) {
  object_model_.update_state(
      x.tail<2 * OBJECT_DIMENSION + CONTACT_STATE>().head<1>());
  return object_model_.get_pose("handle_link");
}

geometry_msgs::PoseStamped ManipulationTrackingController::get_pose_base(
    const mppi::observation_t& x) {
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

geometry_msgs::PoseStamped
ManipulationTrackingController::get_pose_end_effector_ros(
    const Eigen::VectorXd& x) {
  geometry_msgs::PoseStamped pose_ros;
  pose_ros.header.frame_id = "world";
  pose_ros.header.stamp = ros::Time::now();
  mppi_pinocchio::Pose pose = get_pose_end_effector(x);
  mppi_pinocchio::to_msg(pose, pose_ros.pose);
  return pose_ros;
}

geometry_msgs::PoseStamped ManipulationTrackingController::get_pose_handle_ros(
    const Eigen::VectorXd& x) {
  geometry_msgs::PoseStamped pose_ros;
  pose_ros.header.frame_id = "world";
  pose_ros.header.stamp = ros::Time::now();
  mppi_pinocchio::Pose pose = get_pose_handle(x);
  mppi_pinocchio::to_msg(pose, pose_ros.pose);
  return pose_ros;
}

void ManipulationTrackingController::publish_ros() {
  // update robot state visualization
  static int robot_dof =
      std::static_pointer_cast<PandaRaisimDynamics>(model_)->robot_dof_;
  joint_state_.header.stamp = ros::Time::now();
  for (size_t j = 0; j < robot_dof; j++) {
    joint_state_.position[j] = x_(j);
    joint_state_.velocity[j] = x_(j + robot_dof);
  }
  state_publisher_.publish(joint_state_);

  // update object state visualization
  object_state_.header.stamp = ros::Time::now();
  object_state_.position[0] = x_(2 * robot_dof);
  object_state_publisher_.publish(object_state_);

  // visualize contact forces
  std::vector<force_t> forces =
      std::static_pointer_cast<PandaRaisimDynamics>(model_)
          ->get_contact_forces();
  visualization_msgs::MarkerArray force_markers;
  for (const auto& force : forces) {
    force_marker_.points.resize(2);
    force_marker_.points[0].x = force.position(0);
    force_marker_.points[0].y = force.position(1);
    force_marker_.points[0].z = force.position(2);
    force_marker_.points[1].x = force.position(0) + force.force(0) / 10.0;
    force_marker_.points[1].y = force.position(1) + force.force(1) / 10.0;
    force_marker_.points[1].z = force.position(2) + force.force(2) / 10.0;
    force_markers.markers.push_back(force_marker_);
  }
  contact_forces_publisher_.publish(force_markers);

  // publish end effector pose
  Eigen::Vector3d ee_position;
  Eigen::Quaterniond ee_orientation;
  std::static_pointer_cast<PandaRaisimDynamics>(model_)->get_end_effector_pose(
      ee_position, ee_orientation);
  geometry_msgs::PoseStamped pose_ros;
  pose_ros.header.stamp = ros::Time::now();
  pose_ros.header.frame_id = "world";
  pose_ros.pose.position.x = ee_position.x();
  pose_ros.pose.position.y = ee_position.y();
  pose_ros.pose.position.z = ee_position.z();
  pose_ros.pose.orientation.x = ee_orientation.x();
  pose_ros.pose.orientation.y = ee_orientation.y();
  pose_ros.pose.orientation.z = ee_orientation.z();
  pose_ros.pose.orientation.w = ee_orientation.w();
  ee_publisher_.publish(pose_ros);

  // publish handle pose
  Eigen::Vector3d handle_position;
  Eigen::Quaterniond handle_orientation;
  std::static_pointer_cast<PandaRaisimDynamics>(model_)->get_handle_pose(
      handle_position, handle_orientation);
  geometry_msgs::PoseStamped handle_pose;
  handle_pose.header.stamp = ros::Time::now();
  handle_pose.header.frame_id = "world";
  handle_pose.pose.position.x = handle_position.x();
  handle_pose.pose.position.y = handle_position.y();
  handle_pose.pose.position.z = handle_position.z();
  handle_pose.pose.orientation.x = handle_orientation.x();
  handle_pose.pose.orientation.y = handle_orientation.y();
  handle_pose.pose.orientation.z = handle_orientation.z();
  handle_pose.pose.orientation.w = handle_orientation.w();
  handle_publisher_.publish(handle_pose);

  obstacle_marker_publisher_.publish(obstacle_marker_);

  optimal_path_.header.stamp = ros::Time::now();
  optimal_path_.poses.clear();

  optimal_base_path_.header.stamp = ros::Time::now();
  optimal_base_path_.poses.clear();

  mppi_pinocchio::Pose pose_temp;
  solver_->get_optimal_rollout(x_opt_, u_opt_);

  for (const auto& x : x_opt_) {
    optimal_path_.poses.push_back(get_pose_end_effector_ros(x));
    if (!fixed_base_) optimal_base_path_.poses.push_back(get_pose_base(x));
  }

  optimal_trajectory_publisher_.publish(optimal_path_);

  if (!fixed_base_)
    optimal_base_trajectory_publisher_.publish(optimal_base_path_);

  // extrapolate base twist from optimal base path
  if (optimal_base_path_.poses.size() > 2) {
    geometry_msgs::TwistStamped base_twist_from_path;
    base_twist_from_path.header.frame_id = "world";
    base_twist_from_path.twist.linear.x =
        (optimal_base_path_.poses[1].pose.position.x -
         optimal_base_path_.poses[0].pose.position.x) /
        config_.step_size;
    base_twist_from_path.twist.linear.y =
        (optimal_base_path_.poses[1].pose.position.y -
         optimal_base_path_.poses[0].pose.position.y) /
        config_.step_size;
    base_twist_from_path.twist.linear.z = 0.0;

    base_twist_from_path.twist.angular.x = 0.0;
    base_twist_from_path.twist.angular.y = 0.0;
    base_twist_from_path.twist.angular.z =
        (x_opt_[1](3) - x_opt_[0](3)) / config_.step_size;
    base_twist_from_path_publisher_.publish(base_twist_from_path);
  }

  // for debug
  pose_handle_publisher_.publish(get_pose_handle_ros(x_opt_[0]));
}
