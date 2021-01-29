//
// Created by giuseppe on 25.01.21.
//

#include "mppi_manipulation/ros/state_observer.h"

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Geometry>

#include <tf2_ros/static_transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

#include <eigen_conversions/eigen_kdl.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <memory>

namespace manipulation {

StateObserver::StateObserver(const ros::NodeHandle& nh)
    : nh_(nh), tf2_listener_(tf2_buffer_) {

  std::string base_twist_topic;
  nh_.param<std::string>("base_twist_topic", base_twist_topic, "/twist");
  base_twist_subscriber_ = nh_.subscribe(base_twist_topic, 1, &StateObserver::base_pose_callback, this);

  std::string arm_state_topic;
  nh_.param<std::string>("arm_state_topic", arm_state_topic, "/joint_state");
  arm_state_subscriber_ = nh_.subscribe(base_twist_topic, 1, &StateObserver::arm_state_callback, this);

  // ros publishing
  base_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/observer/base_pose", 1);
  object_state_publisher_ = nh_.advertise<sensor_msgs::JointState>("/object/joint_state", 1);

  object_state_.name.push_back("articulation_joint");
  object_state_.position.push_back(0.0);
  object_state_.velocity.push_back(0.0);
  articulation_first_computation_ = true;


}

bool StateObserver::initialize() {
  if(!nh_.param<bool>("fixed_base", fixed_base_, true)){
    ROS_ERROR("Failed to parse fixed_base param.");
    return false;
  }
  if (fixed_base_) state_ = Eigen::VectorXd::Zero(27);
  else state_ = Eigen::VectorXd::Zero(21);

  KDL::Tree object_kinematics;
  if (!kdl_parser::treeFromParam("object_description", object_kinematics)) {
    ROS_ERROR("Failed to create KDL::Tree from 'object_description'");
    return false;
  }

  KDL::Chain chain;
  object_kinematics.getChain("shelf", "handle_link", chain);
  if (chain.getNrOfJoints() != 1) {
    ROS_ERROR("The object has more then one joint. Only one joint supported!");
    return false;
  }

  // at start-up door is closed
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  KDL::JntArray joint_pos(chain.getNrOfJoints());

  // required to calibrate the initial shelf position
  KDL::Frame T_shelf_handle_KDL;
  fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);
  fk_solver->JntToCart(joint_pos, T_shelf_handle_KDL);
  tf::transformKDLToEigen(T_shelf_handle_KDL.Inverse(), T_handle0_shelf_);

  // required to now the origin hinge position
  KDL::Frame T_door_handle_KDL;
  object_kinematics.getChain("door_hinge", "handle_link", chain);
  fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);
  fk_solver->JntToCart(joint_pos, T_door_handle_KDL);
  tf::transformKDLToEigen(T_door_handle_KDL.Inverse(), T_handle0_hinge_);

  ROS_INFO("Robot observer correctly initialized.");
  return true;
}

void StateObserver::update() {
  if (!fixed_base_) {
    state_.head<3>() = base_state_;
    state_.segment<9>(3) = q_;
    state_.segment<3>(12) = base_twist_;
    state_.segment<9>(15) = dq_;
    state_(24) = object_state_.position[0];
    state_(25) = object_state_.velocity[0];
  } else {
    state_.head<9>() = q_;
    state_.tail<9>(9) = dq_;
    state_(18) = object_state_.position[0];
    state_(19) = object_state_.velocity[0];
  }
}

void StateObserver::base_pose_callback(const geometry_msgs::TransformStampedConstPtr& msg) {
  base_state_.x() = msg->transform.translation.x;
  base_state_.y() = msg->transform.translation.y;

  // set to zero pitch and roll components and normalize.
  Eigen::Matrix3d m(Eigen::Quaterniond(msg->transform.rotation.w, msg->transform.rotation.x,
                                       msg->transform.rotation.y, msg->transform.rotation.z));

  Eigen::Vector3d world_ix = m.col(0);  // 2d projection of forward motion axis
  base_state_.z() = std::atan2(world_ix.y(), world_ix.x());
}

void StateObserver::base_twist_callback(const geometry_msgs::TwistConstPtr& msg) {
  std::unique_lock<std::mutex> lock(base_twist_mutex_);
  base_twist_.x() = msg->linear.x;
  base_twist_.y() = msg->linear.y;
  base_twist_.z() = msg->angular.z;
  base_twist_received_ = true;
}

void StateObserver::arm_state_callback(const sensor_msgs::JointStateConstPtr& msg) {
  for (size_t i = 0; i < 9; i++) q_(i) = msg->position[i];
  for (size_t i = 0; i < 9; i++) dq_(i) = msg->velocity[i];
}

void StateObserver::object_pose_callback(const geometry_msgs::TransformStampedConstPtr& msg) {
  tf::transformMsgToEigen(msg->transform, T_world_handle_);
  if (articulation_first_computation_) {
    T_world_shelf_ = T_world_handle_ * T_handle0_shelf_;
    T_hinge_world_ = (T_world_handle_ * T_handle0_hinge_).inverse();
    T_hinge_handle_init_ = T_hinge_world_ * T_world_handle_;

    start_relative_angle_ = std::atan2(T_hinge_handle_init_.translation().x(),
                                       T_hinge_handle_init_.translation().y());
    articulation_first_computation_ = false;
    previous_time_ = ros::Time::now().toSec();

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped T_world_shelf_ros;
    tf::transformEigenToMsg(T_hinge_world_.inverse(), T_world_shelf_ros.transform);
    T_world_shelf_ros.header.stamp = ros::Time::now();
    T_world_shelf_ros.header.frame_id = "world";
    T_world_shelf_ros.child_frame_id = "shelf";
    static_broadcaster.sendTransform(T_world_shelf_ros);
    ROS_INFO("Published initial transform from world to shelf frame");
    return;
  }

  T_hinge_handle_ = T_hinge_world_ * T_world_handle_;

  current_relative_angle_ = std::atan2(T_hinge_handle_.translation().x(),
                                       T_hinge_handle_.translation().y());

  double theta_new = current_relative_angle_ - start_relative_angle_;
  double current_time = ros::Time::now().toSec();
  object_state_.velocity[0] = (theta_new - object_state_.position[0]) / (current_time - previous_time_);
  object_state_.position[0] = theta_new;
  previous_time_ = current_time;
}

void StateObserver::publish() {
  if (!fixed_base_) {
    geometry_msgs::PoseStamped base_pose;
    base_pose.header.stamp = ros::Time::now();
    base_pose.header.frame_id = "world";
    base_pose.pose.position.x = base_state_.x();
    base_pose.pose.position.y = base_state_.y();
    base_pose.pose.position.z = 0.0;
    Eigen::Quaterniond q(Eigen::AngleAxisd(base_state_.z(), Eigen::Vector3d::UnitZ()));
    base_pose.pose.orientation.x = q.x();
    base_pose.pose.orientation.y = q.y();
    base_pose.pose.orientation.z = q.z();
    base_pose.pose.orientation.w = q.w();
    base_pose_publisher_.publish(base_pose);
  }

  object_state_publisher_.publish(object_state_);
}

}  // namespace manipulation
