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
    : nh_(nh), PandaRaisimDynamics(params, true) {  

  state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);
  contact_forces_publisher_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/contact_forces", 10);
  ee_publisher_ =
      nh_.advertise<geometry_msgs::PoseStamped>("/end_effector", 10);
  tau_ext_publisher_ =
      nh_.advertise<std_msgs::Float64MultiArray>("/tau_ext", 1);

  //target
  cylinder_target_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("/target_cylinder/joint_states", 10);
  cylinder_target_trans.header.frame_id = "world";
  cylinder_target_trans.child_frame_id = "cylinder_t_frame";

  //mugs
  mugs_size = 2;
  mug_frame_trans.resize(mugs_size);

  for(int i = 0 ; i < mugs_size; i++)
  {
    mug_frame_trans[i].header.frame_id = "world";
    mug_frame_trans[i].child_frame_id = "mug_frame_" + std::to_string(i);
  }

  //table 
  table_state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("/table/joint_state", 10);
  table_trans.header.frame_id = "world";
  table_trans.child_frame_id = "table_frame";

  joint_state_.name = {
      "panda_joint1", "panda_joint2",        "panda_joint3",
      "panda_joint4", "panda_joint5",        "panda_joint6",
      "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"};

  joint_state_.position.resize(joint_state_.name.size());
  joint_state_.velocity.resize(joint_state_.name.size());
  joint_state_.header.frame_id = "world";

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
}

void ManipulatorDynamicsRos::reset_to_default() {
  x_.setZero();
  x_.head<BASE_ARM_GRIPPER_DIM>() << 0.0, -0.52, 0.0, -1.785,
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

  // update cylinder target state and its visulization
  cylinder_target_.header.stamp = ros::Time::now();
  cylinder_target_trans.header.stamp = ros::Time::now();
  cylinder_target_trans.transform.translation.x = params_.target_pos[0];
  cylinder_target_trans.transform.translation.y = params_.target_pos[1];
  cylinder_target_trans.transform.translation.z = params_.target_pos[2];
  tf2::Quaternion q_cylinder_t;
  q_cylinder_t.setRPY(0, 0, 0);
  cylinder_target_trans.transform.rotation.x = q_cylinder_t.x();
  cylinder_target_trans.transform.rotation.y = q_cylinder_t.y();
  cylinder_target_trans.transform.rotation.z = q_cylinder_t.z();
  cylinder_target_trans.transform.rotation.w = q_cylinder_t.w();

  // update mug frames and its visulization
  mug_frame_trans[0].header.stamp = ros::Time::now();
  mug_frame_trans[0].transform.translation.x = x_(2 * robot_dof_);
  mug_frame_trans[0].transform.translation.y = x_(2 * robot_dof_+1);
  mug_frame_trans[0].transform.translation.z = x_(2 * robot_dof_+2);
  mug_frame_trans[0].transform.rotation.x = x_(2* robot_dof_+3);
  mug_frame_trans[0].transform.rotation.y = x_(2* robot_dof_+4);
  mug_frame_trans[0].transform.rotation.z = x_(2* robot_dof_+5);
  mug_frame_trans[0].transform.rotation.w = x_(2* robot_dof_+6);

  mug_frame_trans[1].header.stamp = ros::Time::now();
  mug_frame_trans[1].transform.translation.x = 0.5;
  mug_frame_trans[1].transform.translation.y = -0.5;
  mug_frame_trans[1].transform.translation.z = 0.1;
  mug_frame_trans[1].transform.rotation.x = 0;
  mug_frame_trans[1].transform.rotation.y = 0;
  mug_frame_trans[1].transform.rotation.z = 0;
  mug_frame_trans[1].transform.rotation.w = 1;
  
  // update table state and its visulization
  table_state_.header.stamp = ros::Time::now();
  table_trans.header.stamp = ros::Time::now();
  table_trans.transform.translation.x = params_.table_position[0];  //TODO (boyang): table state is hardcoded, good for now
  table_trans.transform.translation.y = params_.table_position[1];
  table_trans.transform.translation.z = params_.table_position[2];
  tf2::Quaternion q_table;
  q_table.setRPY(0, 0, 0);
  table_trans.transform.rotation.x = q_table.x();
  table_trans.transform.rotation.y = q_table.y();
  table_trans.transform.rotation.z = q_table.z();
  table_trans.transform.rotation.w = q_table.w();
  
  //publish the joint state and transform

  cylinder_target_publisher_.publish(cylinder_target_);
  broadcaster.sendTransform(cylinder_target_trans);

  // publish mug frames
  for(int i = 0 ; i < mugs_size; i++){
    broadcaster.sendTransform(mug_frame_trans[i]);}

  // publish table frame
  table_state_publisher_.publish(table_state_);
  broadcaster.sendTransform(table_trans);

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

  // publish external torques
  get_external_torque(tau_ext_);
  for (size_t i = 0; i < get_panda()->getDOF(); i++) {
    tau_ext_msg_.data[i] = tau_ext_[i];
  }
  tau_ext_publisher_.publish(tau_ext_msg_);

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

}

}  // namespace manipulation
