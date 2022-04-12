/*!
 * @file     controller_ros.cpp
 * @author   Giuseppe Rizzi
 * @date     03.09.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_manipulation/controller_interface.h"
#include <mppi_pinocchio/ros_conversions.h>
#include "mppi_manipulation/cost.h"
#include "mppi_manipulation/dynamics.h"
#include "mppi_manipulation/params/cost_params.h"
#include "mppi_manipulation/params/dynamics_params.h"

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/package.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <mppi/policies/gaussian_policy.h>
#include <mppi/policies/spline_policy.h>

using namespace manipulation;

bool PandaControllerInterface::init_ros() {
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
      "/mode", 10, &PandaControllerInterface::mode_callback, this);
  ee_pose_desired_subscriber_ =
      nh_.subscribe("/end_effector_pose_desired", 10,
                    &PandaControllerInterface::ee_pose_desired_callback, this);
  ref_subscriber = nh_.subscribe("/controller/set_mppi_reference", 10, &PandaControllerInterface::reference_callback, this);
  std::vector<double> default_pose;
  if (!nh_.param<std::vector<double>>("default_pose", default_pose, {}) ||
      default_pose.size() != 7) {
    ROS_ERROR("Failed to parse the default pose or wrong params.");
    return false;
  }
  default_pose_.setZero(7);
  for (int i = 0; i < 7; i++) {
    default_pose_[i] = default_pose[i];
  }

  if (!nh_.param<double>("object_tolerance", object_tolerance_, 0.0) ||
      object_tolerance_ < 0) {
    ROS_ERROR("Failed to parse the object_tolerance or wrong params.");
    return false;
  }

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

  std::string references_file;
  nh_.param<std::string>("references_file", references_file, "");
  reference_scheduler_.parse_from_file(references_file);

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

void PandaControllerInterface::init_model(
    const std::string& robot_description,
    const std::string& object_description) {
  robot_model_.init_from_xml(robot_description);
  object_model_.init_from_xml(object_description);
  ROS_INFO("[PandaControllerInterface::init_model] ok!");
}

bool PandaControllerInterface::set_controller(mppi::solver_ptr& controller) {
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
  // dynamics
  // -------------------------------
  mppi::dynamics_ptr dynamics;
  if (!dynamics_params_.init_from_ros(nh_)) {
    ROS_ERROR("Failed to init dynamics parameters.");
    return false;
  };
  ROS_INFO_STREAM("Successfully parsed controller dynamics parameters: "
                  << dynamics_params_);
  dynamics = std::make_shared<PandaRaisimDynamics>(dynamics_params_);

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

  double rollouts;
  if (nh_.param<double>("rollouts", rollouts, 0.0)){
    ROS_INFO_STREAM("Overriding default rollouts to " << rollouts);
    config_.rollouts = rollouts;
  }

  // -------------------------------
  // cost
  // -------------------------------
  CostParams cost_params;
  if (!cost_params.init_from_ros(nh_)) {
    ROS_ERROR("Failed to parse cost parameters.");
    return false;
  }

  auto cost = std::make_shared<PandaCost>(cost_params);
  local_cost_ = std::make_unique<manipulation::PandaCost>(cost_params);

  // -------------------------------
  // policy
  // -------------------------------
  std::shared_ptr<mppi::Policy> policy;
  bool gaussian_policy = false;
  nh_.param<bool>("gaussian_policy", gaussian_policy, true);
  if (gaussian_policy) {
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

bool PandaControllerInterface::init_reference_to_current_pose(
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

double PandaControllerInterface::get_stage_cost(const mppi::observation_t& x,
                                                const mppi::input_t& u,
                                                const double t) {
  if (!reference_set_) return -1.0;
  return local_cost_->get_stage_cost(x, u, t);
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
  local_cost_->set_reference_trajectory(ref_);
  reference_set_ = true;
}

void PandaControllerInterface::reference_callback(const std_msgs::Float64MultiArray &msg) {
  std::unique_lock<std::mutex> lock(reference_mutex_);
  mppi::reference_trajectory_t reference_trajectory;
  int size = msg.layout.dim[0].size;
  Eigen::VectorXd reference_t(size);
  for (int i = 0; i < size; i++) reference_t(i) = msg.data[i];
  reference_trajectory.rr = std::vector<Eigen::VectorXd>{reference_t};
  reference_trajectory.tt = std::vector<double>{0.};

  ref_ = mppi::reference_trajectory_t(reference_trajectory);
  get_controller()->set_reference_trajectory(ref_);
  local_cost_->set_reference_trajectory(ref_);
  reference_set_ = true;
}

std::map<std::string, double> PandaControllerInterface::
    get_cost_map(const mppi::observation_t& x, const mppi::input_t& u, const double t) {
  if (!reference_set_) return std::map<std::string, double>{};
  local_cost_->get_stage_cost(x, u, t);
  return local_cost_->get_cost_map();
}

void PandaControllerInterface::mode_callback(
    const std_msgs::Int64ConstPtr& msg) {
  std::unique_lock<std::mutex> lock(reference_mutex_);
  ref_.rr[0](11) = msg->data;
  get_controller()->set_reference_trajectory(ref_);
  local_cost_->set_reference_trajectory(ref_);
  reference_set_ = true;
  ROS_INFO_STREAM("Switching to mode: " << msg->data);
}

void PandaControllerInterface::update_reference(const mppi::observation_t& x,
                                                const double t) {
  if (reference_scheduler_.has_reference(t)) {
    reference_scheduler_.set_reference(t, ref_);

    std::unique_lock<std::mutex> lock(reference_mutex_);
    get_controller()->set_reference_trajectory(ref_);
    local_cost_->set_reference_trajectory(ref_);
    reference_set_ = true;
  }
}

mppi_pinocchio::Pose PandaControllerInterface::get_pose_end_effector(
    const Eigen::VectorXd& x) {
    robot_model_.update_state(x.head<BASE_ARM_GRIPPER_DIM>());
  return robot_model_.get_pose("panda_grasp");
}

mppi_pinocchio::Pose PandaControllerInterface::get_pose_handle(
    const Eigen::VectorXd& x) {
  object_model_.update_state(
      x.tail<2 * OBJECT_DIMENSION + CONTACT_STATE>().head<1>());
  return object_model_.get_pose(dynamics_params_.object_handle_link);
}

geometry_msgs::PoseStamped PandaControllerInterface::get_pose_base(
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

geometry_msgs::PoseStamped PandaControllerInterface::get_pose_end_effector_ros(
    const Eigen::VectorXd& x) {
  geometry_msgs::PoseStamped pose_ros;
  pose_ros.header.frame_id = "world";
  pose_ros.header.stamp = ros::Time::now();
  mppi_pinocchio::Pose pose = get_pose_end_effector(x);
  mppi_pinocchio::to_msg(pose, pose_ros.pose);
  return pose_ros;
}

geometry_msgs::PoseStamped PandaControllerInterface::get_pose_handle_ros(
    const Eigen::VectorXd& x) {
  geometry_msgs::PoseStamped pose_ros;
  pose_ros.header.frame_id = "world";
  pose_ros.header.stamp = ros::Time::now();
  mppi_pinocchio::Pose pose = get_pose_handle(x);
  mppi_pinocchio::to_msg(pose, pose_ros.pose);
  return pose_ros;
}

void PandaControllerInterface::publish_ros() {
  obstacle_marker_publisher_.publish(obstacle_marker_);

  optimal_path_.header.stamp = ros::Time::now();
  optimal_path_.poses.clear();

  optimal_base_path_.header.stamp = ros::Time::now();
  optimal_base_path_.poses.clear();

  mppi_pinocchio::Pose pose_temp;
  get_controller()->get_optimal_rollout(x_opt_, u_opt_);

  for (const auto& x : x_opt_) {
    optimal_path_.poses.push_back(get_pose_end_effector_ros(x));
    optimal_base_path_.poses.push_back(get_pose_base(x));
  }

  optimal_trajectory_publisher_.publish(optimal_path_);
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

void PandaControllerInterface::print_reference() const {
  ROS_INFO_STREAM_THROTTLE(0.5, "First ref entry: " << ref_.rr[0].transpose());
}