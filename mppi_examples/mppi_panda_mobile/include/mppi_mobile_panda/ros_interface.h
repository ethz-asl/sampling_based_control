/*!
 * @file     mppi_ros_interface.h
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#pragma once
#include "mppi_mobile_panda/cost.h"
#include <ros/ros.h>
#include <Eigen/Core>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

namespace mobile_panda{

class PandaRosInterface{
 public:
  explicit PandaRosInterface(ros::NodeHandle& nodeHandle): nh(nodeHandle) {
    state_publisher = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    cost_publisher = nh.advertise<std_msgs::Float64>("/cost", 10);
    end_effector_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/end_effector", 10);
    rollout_cost_publisher = nh.advertise<std_msgs::Float64>("/rollout_cost", 10);
    end_effector_pose_subscriber = nh.subscribe("/end_effector_pose_desired", 10,
        &PandaRosInterface::ee_pose_desired_callback, this);
    obstacle_subscriber = nh.subscribe("/obstacle", 10, &PandaRosInterface::obstacle_callback, this);
    obstacle_marker_publisher = nh.advertise<visualization_msgs::Marker>("/obstacle_marker", 10);
    input_publisher = nh.advertise<std_msgs::Float32MultiArray>("/input", 10);
    joint_state.name = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5",
                        "panda_joint6", "panda_joint7"};
    joint_state.position.resize(7);
    joint_state.header.frame_id = "world";
    input.data.resize(9);

    // obstacle marker
    obstacle_marker.header.frame_id = "world";
    obstacle_marker.type = visualization_msgs::Marker::SPHERE;
    obstacle_marker.color.r = 1.0;
    obstacle_marker.color.g = 0.0;
    obstacle_marker.color.b = 0.0;
    obstacle_marker.color.a = 0.4;
    obstacle_marker.scale.x = 0.1;
    obstacle_marker.scale.y = 0.1;
    obstacle_marker.scale.z = 0.1;
    obstacle_marker.pose.orientation.x = 0.0;
    obstacle_marker.pose.orientation.y = 0.0;
    obstacle_marker.pose.orientation.z = 0.0;
    obstacle_marker.pose.orientation.w = 1.0;

    // base transform
    world_base_tf.header.stamp = ros::Time::now();
    world_base_tf.header.frame_id = "world";
    world_base_tf.child_frame_id = "base";
    world_base_tf.transform.translation.z = 0.0;
    world_base_tf.transform.rotation.x = 0.0;
    world_base_tf.transform.rotation.y = 0.0;
    world_base_tf.transform.rotation.z = 0.0;
    world_base_tf.transform.rotation.w = 1.0;
  };

  ~PandaRosInterface() = default;
 public:
  void publish_state(const Eigen::VectorXd& x){
    joint_state.header.stamp = ros::Time::now();
    for(size_t i=0; i<7; i++)
      joint_state.position[i] = x(i);
    this->state_publisher.publish(joint_state);
    world_base_tf.header.stamp = ros::Time::now();
    world_base_tf.transform.translation.x = x(7);
    world_base_tf.transform.translation.y = x(8);
    tf2::Quaternion q;
    q.setRPY(0, 0, x(9));
    world_base_tf.transform.rotation.x = q.x();
    world_base_tf.transform.rotation.y = q.y();
    world_base_tf.transform.rotation.z = q.z();
    world_base_tf.transform.rotation.w = q.w();
    this->tf_broadcaster.sendTransform(world_base_tf);
  }

  void publish_cost(double c){
    cost.data = c;
    this->cost_publisher.publish(cost);
  }

  void publish_rollout_cost(double c){
    rollout_cost.data = c;
    this->rollout_cost_publisher.publish(rollout_cost);
  }

  bool is_reference_set(){
    return reference_received;
  }

  bool is_obstacle_set(){
    return obstacle_received;
  }

  bool get_reference(pinocchio::SE3& p){
    if (is_reference_set()){
      p = ee_pose_desired;
      reference_received = false;
      return true;
    }
    return false;
  }

  bool get_obstacle(pinocchio::SE3& p){
    if (is_obstacle_set()){
      p = obstacle_pose;
      obstacle_received = false;
      return true;
    }
    return false;
  }

  void ee_pose_desired_callback(const geometry_msgs::PoseStampedConstPtr& msg){
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    q.x() = msg->pose.orientation.x;
    q.y() = msg->pose.orientation.y;
    q.z() = msg->pose.orientation.z;
    q.w() = msg->pose.orientation.w;
    t(0) = msg->pose.position.x;
    t(1) = msg->pose.position.y;
    t(2) = msg->pose.position.z;
    ee_pose_desired = pinocchio::SE3(q, t);
    reference_received = true;
  }

  void obstacle_callback(const geometry_msgs::PoseStampedConstPtr& msg){
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
    q.x() = 0;
    q.y() = 0;
    q.z() = 0;
    q.w() = 1;
    t(0) = msg->pose.position.x;
    t(1) = msg->pose.position.y;
    t(2) = msg->pose.position.z;
    obstacle_pose = pinocchio::SE3(q, t);
    obstacle_received = true;
    obstacle_ever_received = true;
  }

  void publish_end_effector_pose(const pinocchio::SE3& p){
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "panda_link0";
    pose.pose.position.x = p.translation()(0);
    pose.pose.position.y = p.translation()(1);
    pose.pose.position.z = p.translation()(2);
    Eigen::Quaterniond q(p.rotation());
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    end_effector_pose_publisher.publish(pose);
  }

  void publish_obstacle_marker(const double radius){
    if (obstacle_ever_received){
      obstacle_marker.scale.x = radius;
      obstacle_marker.scale.y = radius;
      obstacle_marker.scale.z = radius;
      obstacle_marker.pose.position.x = obstacle_pose.translation()(0);
      obstacle_marker.pose.position.y = obstacle_pose.translation()(1);
      obstacle_marker.pose.position.z = obstacle_pose.translation()(2);
      obstacle_marker_publisher.publish(obstacle_marker);
    }
  }

  void publish_input(const Eigen::VectorXd& u){
    for(size_t i=0; i<9; i++){
      input.data[i] = u(i);
    }
    input_publisher.publish(input);
  }

 private:
  ros::NodeHandle nh;
  ros::Publisher state_publisher;
  ros::Publisher cost_publisher;
  ros::Publisher rollout_cost_publisher;
  ros::Publisher end_effector_pose_publisher;
  ros::Publisher input_publisher;

  bool reference_received = false;
  ros::Subscriber end_effector_pose_subscriber;
  pinocchio::SE3 ee_pose_desired;

  bool obstacle_ever_received = false;
  bool obstacle_received = false;
  ros::Subscriber obstacle_subscriber;
  pinocchio::SE3 obstacle_pose;
  ros::Publisher obstacle_marker_publisher;
  visualization_msgs::Marker obstacle_marker;

  sensor_msgs::JointState joint_state;
  std_msgs::Float64 cost;
  std_msgs::Float64 rollout_cost;
  std_msgs::Float32MultiArray input;

  geometry_msgs::TransformStamped world_base_tf;
  tf2_ros::TransformBroadcaster tf_broadcaster;

};
}
