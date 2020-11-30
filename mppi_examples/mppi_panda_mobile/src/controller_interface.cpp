/*!
 * @file     controller_interface.cpp
 * @author   Giuseppe Rizzi
 * @date     25.09.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_panda_mobile/controller_interface.h"
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

void PandaMobileControllerInterface::init_model(
    const std::string& robot_description) {
  pinocchio::urdf::buildModelFromXML(robot_description, model_);
  data_ = pinocchio::Data(model_);
}

bool PandaMobileControllerInterface::set_controller(
    std::shared_ptr<mppi::PathIntegral>& controller) {
  // -------------------------------
  // internal model
  // -------------------------------
  std::string robot_description;
  if (!nh_.param<std::string>("/robot_description", robot_description, "")) {
    throw std::runtime_error("robot_description has not been set!");
  }

  init_model(robot_description);

  // -------------------------------
  // dynamics
  // -------------------------------
  auto dynamics = std::make_shared<PandaMobileDynamics>(robot_description);

  // -------------------------------
  // cost
  // -------------------------------
  double linear_weight = param_io::param(nh_, "linear_weight", 10.0);
  double angular_weight = param_io::param(nh_, "angular_weight", 10.0);
  auto cost = std::make_shared<PandaMobileCost>(
      robot_description, linear_weight, angular_weight, obstacle_radius_);

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

pinocchio::SE3 PandaMobileControllerInterface::get_pose_end_effector(
    const Eigen::VectorXd& x) {
  pinocchio::forwardKinematics(model_, data_, x.head<7>());
  pinocchio::updateFramePlacements(model_, data_);
  Eigen::Matrix3d base_rotation(
      Eigen::AngleAxisd(x(9), Eigen::Vector3d::UnitZ()));
  Eigen::Vector3d base_translation(x(7), x(8), 0.0);
  pinocchio::SE3 base_tf = pinocchio::SE3(base_rotation, base_translation);
  return base_tf.act(data_.oMf[model_.getFrameId("panda_hand")]);
}

geometry_msgs::PoseStamped
PandaMobileControllerInterface::get_pose_end_effector_ros(
    const Eigen::VectorXd& x) {
  pinocchio::SE3 pose = get_pose_end_effector(x);
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

void PandaMobileControllerInterface::publish_ros() {
  if (obstacle_pose_.header.seq != 0) {  // obstacle set at least once
    obstacle_marker_.pose.position = obstacle_pose_.pose.position;
    obstacle_marker_publisher_.publish(obstacle_marker_);
  }

  optimal_path_.header.stamp = ros::Time::now();
  optimal_path_.poses.clear();
  pinocchio::SE3 pose_temp;
  geometry_msgs::PoseStamped pose_temp_ros;
  get_controller()->get_optimal_rollout(x_opt_, u_opt_);

  for (const auto& x : x_opt_) {
    pose_temp = get_pose_end_effector(x);
    pose_temp_ros.pose.position.x = pose_temp.translation()(0);
    pose_temp_ros.pose.position.y = pose_temp.translation()(1);
    pose_temp_ros.pose.position.z = pose_temp.translation()(2);
    Eigen::Quaterniond q(pose_temp.rotation());
    pose_temp_ros.pose.orientation.x = q.x();
    pose_temp_ros.pose.orientation.y = q.y();
    pose_temp_ros.pose.orientation.z = q.z();
    pose_temp_ros.pose.orientation.w = q.w();
    optimal_path_.poses.push_back(pose_temp_ros);
  }
  optimal_trajectory_publisher_.publish(optimal_path_);
}