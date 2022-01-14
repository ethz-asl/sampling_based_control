/*!
 * @file    dynamics_ros.cpp
 * @author  Matthias Studiger
 * @date    10.04.2021
 * @version 1.0
 * @brief   description
 */

#include "mppi_omav_interaction/dynamics_ros.h"

namespace omav_interaction {
OMAVVelocityDynamicsRos::OMAVVelocityDynamicsRos(
    const ros::NodeHandle &nh, const std::string &robot_description,
    const std::string &object_description, const double dt)
    : OMAVVelocityDynamics(robot_description, object_description, dt) {
  detailed_publishing_ = true;
  if (detailed_publishing_) {
    vis_publisher_ =
        nh_.advertise<visualization_msgs::Marker>("debug/marker_visualization", 0);
    goal_publisher_ =
        nh_.advertise<visualization_msgs::Marker>("debug/marker_goal", 0);
    obstacle_publisher_ =
        nh_.advertise<visualization_msgs::Marker>("debug/marker_obstacle", 0);
    object_state_publisher_ =
        nh_.advertise<sensor_msgs::JointState>("debug/object/joint_state", 1);
    contact_forces_publisher_ =
        nh_.advertise<visualization_msgs::Marker>("debug/contact_force", 1);
    force_dot_ = nh_.advertise<visualization_msgs::Marker>("debug/dot_force", 1);
    vel_dot_ = nh_.advertise<visualization_msgs::Marker>("debug/dot_vel", 1);

    omav_marker_.header.frame_id = "world";
    omav_marker_.id = 0;
    omav_marker_.action = visualization_msgs::Marker::ADD;
    omav_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    omav_marker_.mesh_resource =
        "package://mppi_omav_interaction/data/meshes/ouzel_inspection_full.dae";
    omav_marker_.scale.x = 1.0;
    omav_marker_.scale.y = 1.0;
    omav_marker_.scale.z = 1.0;
    omav_marker_.color.a = 1.0;
    omav_marker_.color.r = 1.0;
    omav_marker_.color.g = 0.0;
    omav_marker_.color.b = 0.0;

    goal_marker_.header.frame_id = "world";
    goal_marker_.id = 1;
    goal_marker_.action = visualization_msgs::Marker::ADD;
    goal_marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
    goal_marker_.mesh_resource =
        "package://mppi_omav_interaction/data/meshes/ouzel_inspection_full.dae";
    goal_marker_.scale.x = 1.0;
    goal_marker_.scale.y = 1.0;
    goal_marker_.scale.z = 1.0;
    goal_marker_.color.a = 1.0;
    goal_marker_.color.r = 0.0;
    goal_marker_.color.g = 1.0;
    goal_marker_.color.b = 0.0;
    goal_marker_.header.stamp = ros::Time::now();
    goal_marker_.pose.position.x = 10.0;
    goal_marker_.pose.position.y = 10.0;
    goal_marker_.pose.position.z = 10.0;
    goal_marker_.pose.orientation.w = 1.0;
    goal_marker_.pose.orientation.x = 0.0;
    goal_marker_.pose.orientation.y = 0.0;
    goal_marker_.pose.orientation.z = 0.0;

    obstacle_marker_.header.frame_id = "world";
    obstacle_marker_.id = 2;
    obstacle_marker_.action = visualization_msgs::Marker::ADD;
    obstacle_marker_.type = visualization_msgs::Marker::CUBE;
    obstacle_marker_.scale.x = 0.2;
    obstacle_marker_.scale.y = 4.0;
    obstacle_marker_.scale.z = 4.0;
    obstacle_marker_.color.a = 1.0;
    obstacle_marker_.color.r = 1.0;
    obstacle_marker_.color.g = 0.0;
    obstacle_marker_.color.b = 0.0;
    obstacle_marker_.header.stamp = ros::Time::now();
    obstacle_marker_.pose.position.x = 2.2;
    obstacle_marker_.pose.position.y = 0.0;
    obstacle_marker_.pose.position.z = 2.0;
    obstacle_marker_.pose.orientation.w = 1.0;
    obstacle_marker_.pose.orientation.x = 0.0;
    obstacle_marker_.pose.orientation.y = 0.0;
    obstacle_marker_.pose.orientation.z = 0.0;

    object_state_.name = {"articulation_joint"};
    object_state_.position.resize(1);

    omav_interaction::conversions::arrow_initialization(force_marker_);
    omav_interaction::conversions::arrow_initialization(force_dot_marker_);
    omav_interaction::conversions::arrow_initialization(vel_dot_marker_);
    // Change color of velocity marker
    vel_dot_marker_.color.r = 0.0;
    vel_dot_marker_.color.b = 1.0;
  }
}

void OMAVVelocityDynamicsRos::reset_to_default() {
  x_.setZero();
  x_(3) = 1.0;
  x_(22) = 1.0;
  reset(x_);
  ROS_INFO_STREAM("Reset simulation ot default value: " << x_.transpose());
}

void OMAVVelocityDynamicsRos::publish_ros() {
  if (detailed_publishing_) {
    // Update robot state visualization
    omav_marker_.header.stamp = ros::Time::now();
    omav_marker_.pose.position.x = x_(0);
    omav_marker_.pose.position.y = x_(1);
    omav_marker_.pose.position.z = x_(2);
    omav_marker_.pose.orientation.w = x_(3);
    omav_marker_.pose.orientation.x = x_(4);
    omav_marker_.pose.orientation.y = x_(5);
    omav_marker_.pose.orientation.z = x_(6);

    static tf::TransformBroadcaster odom_broadcaster;
    tf::Transform omav_odom;
    omav_odom.setOrigin(tf::Vector3(x_(0), x_(1), x_(2)));
    omav_odom.setRotation(tf::Quaternion(x_(4), x_(5), x_(6), x_(3)));
    odom_broadcaster.sendTransform(tf::StampedTransform(
        omav_odom, ros::Time::now(), "world", "odom_omav"));
    // update object state visualization
    object_state_.header.stamp = ros::Time::now();
    object_state_.position[0] = x_(13);
    object_state_publisher_.publish(object_state_);

    // visualize contact forces
    force_t force = get_contact_forces();
    force_normed_ << force.force.normalized();

    force_marker_.points.resize(2);
    force_marker_.points[0].x = force.position(0);
    force_marker_.points[0].y = force.position(1);
    force_marker_.points[0].z = force.position(2);
    force_marker_.points[1].x = force.position(0) + force.force(0) / 10;
    force_marker_.points[1].y = force.position(1) + force.force(1) / 10;
    force_marker_.points[1].z = force.position(2) + force.force(2) / 10;

    force_dot_marker_.points.resize(2);
    force_dot_marker_.points[0].x = 0;
    force_dot_marker_.points[0].y = 0;
    force_dot_marker_.points[0].z = 0;
    force_dot_marker_.points[1].x = x_(14);
    force_dot_marker_.points[1].y = force_normed_(1);
    force_dot_marker_.points[1].z = force_normed_(2);

    vel_normed_ << x_(10), x_(11), x_(12);
    vel_dot_marker_.points.resize(2);
    vel_dot_marker_.points[0].x = x_(7);
    vel_dot_marker_.points[0].y = x_(8);
    vel_dot_marker_.points[0].z = x_(9);
    vel_dot_marker_.points[1].x = x_(10);
    vel_dot_marker_.points[1].y = x_(11);
    vel_dot_marker_.points[1].z = x_(12);

    force_dot_.publish(force_dot_marker_);
    vel_dot_.publish(vel_dot_marker_);
    contact_forces_publisher_.publish(force_marker_);
    vis_publisher_.publish(omav_marker_);
    goal_publisher_.publish(goal_marker_);
    obstacle_publisher_.publish(obstacle_marker_);
  }
}
} // namespace omav_velocity
