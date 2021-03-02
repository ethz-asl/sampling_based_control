//
// Created by giuseppe on 25.01.21.
//

#include "mppi_royalpanda/state_observer.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Geometry>

#include <tf2_ros/static_transform_broadcaster.h>

#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <manipulation_msgs/conversions.h>

namespace royalpanda {

StateObserver::StateObserver(const ros::NodeHandle& nh) : nh_(nh) {
  std::string base_pose_topic;
  nh_.param<std::string>("base_pose_topic", base_pose_topic, "/base_pose");
  base_pose_subscriber_ = nh_.subscribe(
      base_pose_topic, 1, &StateObserver::base_pose_callback, this);

  std::string base_twist_topic;
  nh_.param<std::string>("base_twist_topic", base_twist_topic, "/base_twist");
  base_twist_subscriber_ = nh_.subscribe(
      base_twist_topic, 1, &StateObserver::base_twist_callback, this);

  std::string arm_state_topic;
  nh_.param<std::string>("arm_state_topic", arm_state_topic, "/arm_state");
  arm_state_subscriber_ = nh_.subscribe(
      arm_state_topic, 1, &StateObserver::arm_state_callback, this);

  std::string object_pose_topic;
  nh_.param<std::string>("object_pose_topic", object_pose_topic,
                         "/object_pose");
  object_pose_subscriber_ = nh_.subscribe(
      object_pose_topic, 1, &StateObserver::object_pose_callback, this);

  // ros publishing
  state_publisher_ =
      nh_.advertise<manipulation_msgs::State>("/observer/state", 1);
  base_pose_publisher_ =
      nh_.advertise<geometry_msgs::PoseStamped>("/observer/base_pose", 1);
  base_twist_publisher_ =
      nh_.advertise<geometry_msgs::TwistStamped>("/observer/base_twist", 1);
  object_state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("/observer/object/joint_state", 1);
  robot_state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("/observer/base/joint_state", 1);

  object_state_.name.push_back("articulation_joint");
  object_state_.position.push_back(0.0);
  object_state_.velocity.push_back(0.0);
  articulation_first_computation_ = true;

  robot_state_.name = {"x_base_joint", "y_base_joint", "pivot_joint"};
  robot_state_.position.resize(robot_state_.name.size());
  robot_state_.velocity.resize(robot_state_.name.size());
  robot_state_.header.frame_id = "world";
}

bool StateObserver::initialize() {
  if (!nh_.param<bool>("fixed_base", fixed_base_, true)) {
    ROS_ERROR("Failed to parse fixed_base param.");
    return false;
  }

  if (!nh_.getParam("base_alpha", base_alpha_) || base_alpha_ < 0 ||
      base_alpha_ > 1) {
    ROS_ERROR("Failed to parse base_alpha param or invalid.");
    return false;
  }

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
  KDL::JntArray joint_pos(chain.getNrOfJoints());

  // required to calibrate the initial shelf position
  KDL::Frame T_shelf_handle_KDL;
  KDL::ChainFkSolverPos_recursive fk_solver_shelf(chain);
  fk_solver_shelf.JntToCart(joint_pos, T_shelf_handle_KDL);
  tf::transformKDLToEigen(T_shelf_handle_KDL.Inverse(), T_handle0_shelf_);

  // required to now the origin hinge position
  KDL::Frame T_door_handle_KDL;
  object_kinematics.getChain("axis_link", "handle_link", chain);
  KDL::ChainFkSolverPos_recursive fk_solver_hinge(chain);
  fk_solver_hinge.JntToCart(joint_pos, T_door_handle_KDL);
  tf::transformKDLToEigen(T_door_handle_KDL.Inverse(), T_handle0_hinge_);

  // get the relative pose of base to reference frame
  KDL::Tree robot_kinematics;
  if (!kdl_parser::treeFromParam("robot_description", robot_kinematics)) {
    ROS_ERROR("Failed to create KDL::Tree from 'robot_description'");
    return false;
  }

  KDL::Chain robot_chain;
  if (!robot_kinematics.getChain("base_link", "reference_link", robot_chain)) {
    ROS_ERROR("Failed to extract chain from base_link to reference_link");
    return false;
  }

  KDL::Frame T_base_reference_KDL;
  KDL::JntArray robot_joint_pos(robot_chain.getNrOfJoints());
  std::cout << "there are: " << robot_chain.getNrOfJoints() << std::endl;
  std::cout << robot_joint_pos.data.transpose() << std::endl;
  KDL::ChainFkSolverPos_recursive robot_solver(robot_chain);
  robot_solver.JntToCart(robot_joint_pos, T_base_reference_KDL);
  tf::transformKDLToEigen(T_base_reference_KDL.Inverse(), T_reference_base_);

  ROS_INFO_STREAM("Static transformations summary: "
                  << std::endl
                  << " T_shelf_handle:\n "
                  << T_handle0_shelf_.inverse().matrix() << std::endl
                  << " T_hinge_handle:\n "
                  << T_handle0_hinge_.inverse().matrix() << std::endl
                  << " T_base_reference:\n "
                  << T_reference_base_.inverse().matrix() << std::endl);
  ROS_INFO("Robot observer correctly initialized.");
  return true;
}

void StateObserver::update() {
  if (!fixed_base_) {
    manipulation::conversions::toMsg(
        base_state_, base_twist_, q_, dq_, object_state_.position[0],
        object_state_.velocity[0], false, state_ros_);
    state_ros_.header.stamp = ros::Time::now();
  } else {
    manipulation::conversions::toMsg(q_, dq_, object_state_.position[0],
                                     object_state_.velocity[0], false,
                                     state_ros_);
    state_ros_.header.stamp = ros::Time::now();
  }
}

void StateObserver::base_pose_callback(const nav_msgs::OdometryConstPtr& msg) {
  tf::poseMsgToEigen(msg->pose.pose, T_world_reference_);
  T_world_base_ = T_world_reference_ * T_reference_base_;

  base_state_.x() = T_world_base_.translation().x();
  base_state_.y() = T_world_base_.translation().y();

  Eigen::Vector3d ix =
      T_world_base_.rotation().col(0);  // 2d projection of forward motion axis
  base_state_.z() = std::atan2(ix.y(), ix.x());
}

void StateObserver::base_twist_callback(const nav_msgs::OdometryConstPtr& msg) {
  Eigen::Vector3d odom_base_twist(msg->twist.twist.linear.x,
                                  msg->twist.twist.linear.y, 0.0);
  odom_base_twist =
      Eigen::AngleAxis(base_state_.z(), Eigen::Vector3d::UnitZ()) *
      odom_base_twist;
  base_twist_ = base_alpha_ * base_twist_ + (1 - base_alpha_) * odom_base_twist;
}

void StateObserver::arm_state_callback(
    const sensor_msgs::JointStateConstPtr& msg) {
  if (msg->name.size() != 9) {
    ROS_WARN_STREAM_THROTTLE(
        2.0, "Joint state has the wrong size: " << msg->name.size());
    return;
  }
  for (size_t i = 0; i < 9; i++) q_(i) = msg->position[i];
  for (size_t i = 0; i < 9; i++) dq_(i) = msg->velocity[i];
}

void StateObserver::object_pose_callback(
    const nav_msgs::OdometryConstPtr& msg) {
  tf::poseMsgToEigen(msg->pose.pose, T_world_handle_);
  if (articulation_first_computation_) {
    ROS_INFO("First computation of the shelf pose.");
    T_world_shelf_ = T_world_handle_ * T_handle0_shelf_;
    T_hinge_world_ = (T_world_handle_ * T_handle0_hinge_).inverse();
    T_hinge_handle_init_ = T_hinge_world_ * T_world_handle_;

    start_relative_angle_ = std::atan2(T_hinge_handle_init_.translation().x(),
                                       T_hinge_handle_init_.translation().y());
    articulation_first_computation_ = false;
    previous_time_ = ros::Time::now().toSec();

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped T_world_shelf_ros;
    tf::transformEigenToMsg(T_world_shelf_, T_world_shelf_ros.transform);
    T_world_shelf_ros.header.stamp = ros::Time::now();
    T_world_shelf_ros.header.frame_id = "world";
    T_world_shelf_ros.child_frame_id = "shelf";
    static_broadcaster.sendTransform(T_world_shelf_ros);
    ROS_INFO_STREAM("Published initial transform from world to shelf frame.");
    return;
  }

  T_hinge_handle_ = T_hinge_world_ * T_world_handle_;
  current_relative_angle_ = std::atan2(T_hinge_handle_.translation().x(),
                                       T_hinge_handle_.translation().y());

  double theta_new = current_relative_angle_ - start_relative_angle_;
  double current_time = ros::Time::now().toSec();

  object_state_.velocity[0] =
      (theta_new - object_state_.position[0]) / (current_time - previous_time_);
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
    Eigen::Quaterniond q(
        Eigen::AngleAxisd(base_state_.z(), Eigen::Vector3d::UnitZ()));
    base_pose.pose.orientation.x = q.x();
    base_pose.pose.orientation.y = q.y();
    base_pose.pose.orientation.z = q.z();
    base_pose.pose.orientation.w = q.w();
    base_pose_publisher_.publish(base_pose);

    robot_state_.position[0] = base_state_.x();
    robot_state_.position[1] = base_state_.y();
    robot_state_.position[2] = base_state_.z();

    base_twist_ros_.header.frame_id = "world";
    base_twist_ros_.header.stamp = ros::Time::now();
    base_twist_ros_.twist.linear.x = base_twist_.x();
    base_twist_ros_.twist.linear.y = base_twist_.y();
    base_twist_ros_.twist.angular.z = base_twist_.z();
    base_twist_publisher_.publish(base_twist_ros_);

    robot_state_.header.stamp = ros::Time::now();
    robot_state_publisher_.publish(robot_state_);
  }

  object_state_.header.stamp = ros::Time::now();
  object_state_publisher_.publish(object_state_);

  state_publisher_.publish(state_ros_);
}

}  // namespace royalpanda
