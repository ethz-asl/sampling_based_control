//
// Created by giuseppe on 18.01.21.
//

#include "mppi_sliding/dynamics_ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <signal_logger/signal_logger.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace manipulation {

ManipulatorDynamicsRos::ManipulatorDynamicsRos(const ros::NodeHandle& nh,
                                               const DynamicsParams& params)
    : nh_(nh), PandaRaisimDynamics(params) {

  std::cout << "In dynamic ros constructor, dynamic.cpp finished, params set up " << std::endl;
  state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
  object_state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("/object/joint_state", 10);
  contact_forces_publisher_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/contact_forces", 10);
  ee_publisher_ =
      nh_.advertise<geometry_msgs::PoseStamped>("/end_effector", 10);
  handle_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/handle", 10);
  tau_ext_publisher_ =
      nh_.advertise<std_msgs::Float64MultiArray>("/tau_ext", 1);
  power_publisher_ = nh_.advertise<std_msgs::Float64>("/power", 1);

  // keypoints markers
  kp_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>( "/keypoints_marker_array", 10);

  // cylinder
  cylinder_state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("/cylinder/joint_state", 10);

  cylinder_trans.header.frame_id = "world";
  cylinder_trans.child_frame_id = "cylinder_frame";

  joint_state_.name = {
      "x_base_joint", "y_base_joint",        "pivot_joint",
      "panda_joint1", "panda_joint2",        "panda_joint3",
      "panda_joint4", "panda_joint5",        "panda_joint6",
      "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"};

  joint_state_.position.resize(joint_state_.name.size());
  joint_state_.velocity.resize(joint_state_.name.size());
  joint_state_.header.frame_id = "world";

  object_state_.name = {params_.articulation_joint};
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
  tau_ext_msg_.data.resize(get_panda()->getDOF());
  ff_tau_.setZero(get_panda()->getDOF());
  integral_term_.setZero(ARM_DIMENSION);


  signal_logger::add(tau_ext_, "ground_truth_external_torque");
  signal_logger::logger->updateLogger();

  std::cout << " End of  dynamic ros constructor, dynamic.cpp finished, params set up " << std::endl;

}

void ManipulatorDynamicsRos::reset_to_default() {
  x_.setZero();
  x_.head<BASE_ARM_GRIPPER_DIM>() << 0.0, 0.0, 0.0, 0.0, -0.52, 0.0, -1.785,
      0.0, 1.10, 0.69, 0.04, 0.04;
  reset(x_, 0.0);
  ROS_INFO_STREAM("Reset simulation ot default value: " << x_.transpose());
}

void ManipulatorDynamicsRos::publish_ros() {
  tweak = tweak + 1;

  // update robot state visualization
  joint_state_.header.stamp = ros::Time::now();
  for (size_t j = 0; j < robot_dof_; j++) {
    joint_state_.position[j] = x_(j);
    joint_state_.velocity[j] = x_(j + robot_dof_);
  }
  state_publisher_.publish(joint_state_);

  // update object state visualization
  object_state_.header.stamp = ros::Time::now();
  object_state_.position[0] = x_(2 * robot_dof_);
  object_state_publisher_.publish(object_state_);

  // update cylinder state and its visulization
  Eigen::Vector3d cylinder_position;
  Eigen::Quaterniond cylinder_orientation;
  cylinder_state_.header.stamp = ros::Time::now();
  
  cylinder_trans.header.stamp = ros::Time::now();
  cylinder_trans.transform.translation.x = x_(2 * robot_dof_);
  cylinder_trans.transform.translation.y = x_(2 * robot_dof_ +1 );
  cylinder_trans.transform.translation.z = 1;
  tf2::Quaternion q;
  q.setRPY(0, 0, x_(2 * robot_dof_)+2);
  cylinder_trans.transform.rotation.x = q.x();
  cylinder_trans.transform.rotation.y = q.y();
  cylinder_trans.transform.rotation.z = q.z();
  cylinder_trans.transform.rotation.w = q.w();

  //send the joint state and transform
  cylinder_state_publisher_.publish(cylinder_state_);
  broadcaster.sendTransform(cylinder_trans);

  // visualize contact forces
  std::vector<force_t> forces = get_contact_forces();
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

  visualization_msgs::MarkerArray kp_markers_;
  int kp_numbers = 10;
  for(int i = 0 ; i < int(kp_numbers); i ++)
  {
    double rand_num = ((double) std::rand() / (RAND_MAX));
    double rand_positive_1 = ((double) std::rand() / (RAND_MAX));
    double rand_positive_2 = ((double) std::rand() / (RAND_MAX));
    int multi_1 = 1;
    int multi_2 = 1;
    int multi_3 = 1;
    if (rand_positive_1 < 0.5)
    {
      multi_1 = -1;
    } 
    if (rand_positive_2 < 0.5)
    {
      multi_2 = -1;
    } 
    if (i > 4)
    {
      multi_3 = -1;
    } 
    // keypoints marker
    kp_marker_.type = visualization_msgs::Marker::SPHERE;
    kp_marker_.id = i;
    kp_marker_.header.frame_id = "world";
    kp_marker_.action = visualization_msgs::Marker::ADD;
    kp_marker_.color.r = 1.0;
    kp_marker_.color.b = 0.0;
    kp_marker_.color.g = 0.0;
    kp_marker_.color.a = 1.0;
    kp_marker_.scale.x = 0.03;
    kp_marker_.scale.y = 0.03;
    kp_marker_.scale.z = 0.03;
    kp_marker_.pose.orientation.x = 0.0;
    kp_marker_.pose.orientation.y = 0.0;
    kp_marker_.pose.orientation.z = 0.0;
    kp_marker_.pose.orientation.w = 1.0;
    kp_marker_.pose.position.z = cylinder_trans.transform.translation.z + multi_3*0.25;
    kp_marker_.pose.position.y = cylinder_trans.transform.translation.y + multi_1*0.1 * rand_num;
    kp_marker_.pose.position.x = cylinder_trans.transform.translation.x + multi_2*0.1 * sqrt(1-rand_num*rand_num) ;
    kp_markers_.markers.push_back(kp_marker_);
  }
  kp_publisher_.publish(kp_markers_);

  // publish external torques
  get_external_torque(tau_ext_);
  for (size_t i = 0; i < get_panda()->getDOF(); i++) {
    tau_ext_msg_.data[i] = tau_ext_[i];
  }
  tau_ext_publisher_.publish(tau_ext_msg_);

  // publish power exchanged
  std_msgs::Float64 power;
  power.data = tau_ext_.transpose() * joint_v_;
  power_publisher_.publish(power);

  // publish end effector pose
  Eigen::Vector3d ee_position;
  Eigen::Quaterniond ee_orientation;
  get_end_effector_pose(ee_position, ee_orientation);
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
  get_handle_pose(handle_position, handle_orientation);
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
}

}  // namespace manipulation
