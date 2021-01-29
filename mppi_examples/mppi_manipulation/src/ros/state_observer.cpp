//
// Created by giuseppe on 25.01.21.
//

#include "mppi_manipulation/ros/state_observer.h"

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Geometry>

#include <eigen_conversions/eigen_msg.h>

#include <eigen_conversions/eigen_kdl.h>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/frameacc_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <memory>

namespace manipulation {

StateObserver::StateObserver(const ros::NodeHandle& nh)
    : nh_(nh), tf2_listener_(tf2_buffer_) {

  std::string base_pose_topic;
  nh_.param<std::string>("base_pose_topic", base_pose_topic, "/base_pose");
  base_pose_subscriber_ = nh_.subscribe(base_pose_topic, 1, &StateObserver::base_pose_callback, this);

  std::string base_twist_topic;
  nh_.param<std::string>("base_twist_topic", base_twist_topic, "/base_twist");
  base_twist_subscriber_ = nh_.subscribe(base_twist_topic, 1, &StateObserver::base_twist_callback, this);

  std::string arm_state_topic;
  nh_.param<std::string>("arm_state_topic", arm_state_topic, "/arm_state");
  arm_state_subscriber_ = nh_.subscribe(arm_state_topic, 1, &StateObserver::arm_state_callback, this);

  std::string object_pose_topic;
  nh_.param<std::string>("object_pose_topic", object_pose_topic, "/object_pose");
  object_pose_subscriber_ = nh_.subscribe(object_pose_topic, 1, &StateObserver::object_pose_callback, this);

  // ros publishing
  base_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/observer/base_pose", 1);
  object_state_publisher_ = nh_.advertise<sensor_msgs::JointState>("/observer/object/joint_state", 1);
  robot_state_publisher_ = nh_.advertise<sensor_msgs::JointState>("/observer/robot/joint_state", 1);


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
  if(!nh_.param<bool>("fixed_base", fixed_base_, true)){
    ROS_ERROR("Failed to parse fixed_base param.");
    return false;
  }
  if (fixed_base_) state_ = Eigen::VectorXd::Zero(27);
  else state_ = Eigen::VectorXd::Zero(21);

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
  fk_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
  fk_solver->JntToCart(joint_pos, T_shelf_handle_KDL);
  tf::transformKDLToEigen(T_shelf_handle_KDL.Inverse(), T_handle0_shelf_);

  // required to now the origin hinge position
  KDL::Frame T_door_handle_KDL;
  object_kinematics.getChain("door_hinge", "handle_link", chain);
  fk_solver.reset(new KDL::ChainFkSolverPos_recursive(chain));
  fk_solver->JntToCart(joint_pos, T_door_handle_KDL);
  tf::transformKDLToEigen(T_door_handle_KDL.Inverse(), T_handle0_hinge_);

  // required to know the exact base origin
  //urdf::Model model;
  //model.initParam("robot_description");
  // kdl_parser::treeFromUrdfModel(model, robot_kinematics);
  if (!kdl_parser::treeFromParam("robot_description", robot_kinematics)) {
    ROS_ERROR("Failed to create KDL::Tree from 'robot_description'");
    return false;
  }

  if (!robot_kinematics.getChain("base_link", "reference_link", robot_chain)){
    ROS_ERROR("Failed to extract chain from base_link to reference_link");
    return false;
  }

  KDL::Frame T_base_reference_KDL;
  KDL::JntArray robot_joint_pos(robot_chain.getNrOfJoints());
  std::cout << "there are: " << robot_chain.getNrOfJoints() << std::endl;
  std::cout << robot_joint_pos.data.transpose() << std::endl;
  fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(robot_chain);
  fk_solver->JntToCart(robot_joint_pos, T_base_reference_KDL);
  ROS_INFO_STREAM("KDL transfrom base to reference is: " << T_base_reference_KDL);
  tf::transformKDLToEigen(T_base_reference_KDL.Inverse(), T_reference_base_);
  ROS_INFO_STREAM("Transform reference to base is: \n" << T_reference_base_.matrix());

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

void StateObserver::base_pose_callback(const nav_msgs::OdometryConstPtr& msg) {
  tf::poseMsgToEigen(msg->pose.pose, T_world_reference_);
  T_world_base_ = T_world_reference_ * T_reference_base_;

  base_state_.x() = T_world_base_.translation().x();
  base_state_.y() = T_world_base_.translation().y();
//
//  // set to zero pitch and roll components and normalize.
//  Eigen::Matrix3d m(Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
//                                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z));

  Eigen::Vector3d world_ix = T_world_base_.rotation().col(0);  // 2d projection of forward motion axis
  base_state_.z() = std::atan2(world_ix.y(), world_ix.x());
}

void StateObserver::base_twist_callback(const nav_msgs::OdometryConstPtr& msg) {
  base_twist_.x() = msg->twist.twist.linear.x;
  base_twist_.y() = msg->twist.twist.linear.y;
  base_twist_.z() = msg->twist.twist.angular.z;
}

void StateObserver::arm_state_callback(const sensor_msgs::JointStateConstPtr& msg) {
  for (size_t i = 0; i < 9; i++) q_(i) = msg->position[i];
  for (size_t i = 0; i < 9; i++) dq_(i) = msg->velocity[i];
}

void StateObserver::object_pose_callback(const nav_msgs::OdometryConstPtr& msg) {
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

    robot_state_.position[0] = base_state_.x();
    robot_state_.position[1] = base_state_.y();
    robot_state_.position[2] = base_state_.z();
  }

  object_state_.header.stamp = ros::Time::now();
  object_state_publisher_.publish(object_state_);

  robot_state_.header.stamp = ros::Time::now();
  robot_state_publisher_.publish(robot_state_);

}

}  // namespace manipulation
