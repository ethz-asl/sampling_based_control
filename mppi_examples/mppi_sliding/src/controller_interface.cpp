/*!
 * @file     controller_ros.cpp
 * @author   Giuseppe Rizzi
 * @date     03.09.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_sliding/controller_interface.h"
#include <mppi_pinocchio/ros_conversions.h>
#include "mppi_sliding/cost.h"
#include "mppi_sliding/dynamics.h"
#include "mppi_sliding/params/cost_params.h"
#include "mppi_sliding/params/dynamics_params.h"

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
  // try {
  //   transformStamped = tfBuffer.lookupTransform(
  //       "world", "obstacle", ros::Time(0), ros::Duration(3.0));
  // } catch (tf2::TransformException& ex) {
  //   ROS_WARN("%s", ex.what());
  //   return false;
  // }

  obstacle_marker_.header.frame_id = "world";
  {
    obstacle_marker_.type = visualization_msgs::Marker::CYLINDER;
    obstacle_marker_.color.r = 1.0;
    obstacle_marker_.color.g = 0.0;
    obstacle_marker_.color.b = 0.0;
    obstacle_marker_.color.a = 0.4;
    obstacle_marker_.scale.x = 2.0 * 0.01;
    obstacle_marker_.scale.y = 2.0 * 0.01;
    obstacle_marker_.scale.z = 0.01;
    obstacle_marker_.pose.orientation.x = 0;
    obstacle_marker_.pose.orientation.y = 0;
    obstacle_marker_.pose.orientation.z = 0;
    obstacle_marker_.pose.orientation.w = 1;
    obstacle_marker_.pose.position.x = 100;
    obstacle_marker_.pose.position.y = 100;
    obstacle_marker_.pose.position.z = 10;
  }

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

  // closed-loop object-related units
  cylinder_state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("/cylinder/joint_state", 10);
  cylinder_trans.header.frame_id = "world";
  cylinder_trans.child_frame_id = "cylinder_frame";

  object_predict_publisher_ =  
    nh_.advertise<visualization_msgs::MarkerArray>("/predicted_objects", 10);
  object_predict_marker_.type = visualization_msgs::Marker::CYLINDER;
  object_predict_marker_.header.frame_id = "world";
  object_predict_marker_.action = visualization_msgs::Marker::ADD;
  object_predict_marker_.color.r = 0.0;
  object_predict_marker_.color.b = 1.0;
  object_predict_marker_.color.g = 0.0;
  object_predict_marker_.color.a = 1.0;


  ROS_INFO("[PandaControllerInterface::init_ros] ok!");
  return true;
}

void PandaControllerInterface::init_model(
    const std::string& robot_description,
    const std::string& object_description,
    const std::string& cylinder_description) {
  robot_model_.init_from_xml(robot_description);
  object_model_.init_from_xml(object_description);
  cylinder_model_.init_from_xml(cylinder_description);

  ROS_INFO("[PandaControllerInterface::init_model] ok!");
}

bool PandaControllerInterface::set_controller(mppi::solver_ptr& controller) {
  // -------------------------------
  // internal model
  // -------------------------------
  ROS_INFO("--------------setting controller---------------- ");
  std::string robot_description, object_description, cylinder_description;
  if (!nh_.param<std::string>("/robot_description", robot_description, "")) {
    throw std::runtime_error(
        "Could not parse robot description. Is the parameter set?");
  }
  if (!nh_.param<std::string>("/object_description", object_description, "")) {
    throw std::runtime_error(
        "Could not parse object description. Is the parameter set?");
  }

  if (!nh_.param<std::string>("/cylinder_description", cylinder_description, "")) {
    throw std::runtime_error(
        "Could not parse object description. Is the parameter set?");
  }

  init_model(robot_description, object_description, cylinder_description);

  // -------------------------------
  // dynamics
  // -------------------------------
  ROS_INFO("setting controller dynamics ");
  if (!dynamics_params_.init_from_ros(nh_)) {
    ROS_ERROR("Failed to init dynamics parameters.");
    return false;
  };
  ROS_INFO_STREAM("Successfully parsed controller dynamics parameters: "
                  << dynamics_params_);
  dynamics = std::make_shared<PandaRaisimDynamics>(dynamics_params_,false);

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

  ROS_INFO("setting controller Cost");
  auto cost = std::make_shared<PandaCost>(cost_params);
  local_cost_ = std::make_unique<manipulation::PandaCost>(cost_params);

  // -------------------------------
  // policy
  // -------------------------------
  ROS_INFO("setting controller policy");
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
  ROS_INFO("setting controller solver");
  controller = std::make_shared<mppi::Solver>(dynamics, cost, policy, config_);


  // -------------------------------
  // initialize reference
  // -------------------------------
  ROS_INFO("setting controller, init reference ");
  ref_.rr.resize(1, mppi::observation_t::Zero(PandaDim::REFERENCE_DIMENSION));
  // init obstacle fare away
  
  ref_.rr[0](7) = 100;
  ref_.rr[0](8) = 100;
  ref_.rr[0](9) = 100;
  ref_.rr[0].tail<1>()(0) = 0.0;
  ref_.tt.resize(1, 0.0);
  std::cout << "ref rr size: " << ref_.rr[0].size() << std::endl;
  for(int i = 0 ; i < ref_.rr[0].size(); i ++)
  {
    std::cout << ref_.rr[0](i) << ",";
  }
  
  std::cout << "ref tt size: " << ref_.tt.size() << std::endl;
  ROS_INFO(" -----------------controller set done-------------------");
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

 void PandaControllerInterface::publish_ros_obj(const Eigen::VectorXd& state)
 {
  //ROS_INFO_STREAM("estimated:  " << state );
  cylinder_state_.header.stamp = ros::Time::now();
  cylinder_trans.header.stamp = ros::Time::now();
  cylinder_trans.transform.translation.x = state[0];
  cylinder_trans.transform.translation.y = state[1];
  cylinder_trans.transform.translation.z = state[2];

  cylinder_trans.transform.rotation.x = state[4];
  cylinder_trans.transform.rotation.y = state[5];
  cylinder_trans.transform.rotation.z = state[6];
  cylinder_trans.transform.rotation.w = state[3];

  cylinder_state_publisher_.publish(cylinder_state_);
  broadcaster.sendTransform(cylinder_trans);
 }

void PandaControllerInterface::publish_ros_obj(const mppi::observation_array_t& x_opt_)
{
  int step_num = (x_opt_.size()>50) ? 50 : x_opt_.size();

  object_predict_marker_.scale.x = 0.1;
  object_predict_marker_.scale.y = 0.1;
  object_predict_marker_.scale.z = 0.1;

  obj_state.setZero(OBJECT_DIMENSION);
  object_predict_markers.markers.clear();
  for (int i=0;i<step_num;i++)
  { 
    obj_state = x_opt_[i].segment<OBJECT_DIMENSION>(2*BASE_ARM_GRIPPER_DIM);
    object_predict_marker_.pose.position.x = obj_state[0];
    object_predict_marker_.pose.position.y = obj_state[1];
    object_predict_marker_.pose.position.z = obj_state[2];
    object_predict_marker_.pose.orientation.x = obj_state[4];
    object_predict_marker_.pose.orientation.y = obj_state[5];
    object_predict_marker_.pose.orientation.z = obj_state[6];
    object_predict_marker_.pose.orientation.w = obj_state[3];
    object_predict_marker_.id = i;
    object_predict_marker_.color.a = 1.0 - 3*(i/step_num);
    object_predict_marker_.color.r = 0.0; 
    object_predict_marker_.color.g = 0.2;
    object_predict_marker_.color.b = 0.8;
    object_predict_markers.markers.push_back(object_predict_marker_);
  }

  object_predict_publisher_.publish(object_predict_markers);

}

void PandaControllerInterface::publish_ros() {
  obstacle_marker_publisher_.publish(obstacle_marker_);

  optimal_path_.header.stamp = ros::Time::now();
  optimal_path_.poses.clear();

  // optimal_base_path_.header.stamp = ros::Time::now();
  // optimal_base_path_.poses.clear();

  mppi_pinocchio::Pose pose_temp;
  get_controller()->get_optimal_rollout(x_opt_, u_opt_);

  for (const auto& x : x_opt_) {
    optimal_path_.poses.push_back(get_pose_end_effector_ros(x));
    //optimal_base_path_.poses.push_back(get_pose_base(x));
  }

  obj_state.setZero(7);
  // obj_state = dynamics->get_state().segment<OBJECT_DIMENSION>(2*BASE_ARM_GRIPPER_DIM);
  obj_state = dynamics->get_primitive_state();
  publish_ros_obj(obj_state);
  publish_ros_obj(x_opt_);
  optimal_trajectory_publisher_.publish(optimal_path_);
  // optimal_base_trajectory_publisher_.publish(optimal_base_path_);

  // for debug
  pose_handle_publisher_.publish(get_pose_handle_ros(x_opt_[0]));
}
