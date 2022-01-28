//
// Created by boyang on 09.12.21.
//

#include "mppi_sliding/state_observer.h"
#include "mppi_sliding/utils.h"

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

namespace manipulation_panda {

StateObserver::StateObserver(const ros::NodeHandle& nh)
    : nh_(nh), tf_listener_(tf_buffer_) {
  nh_.param<bool>("simulation", simulation_, false);

  std::string arm_state_topic;
  nh_.param<std::string>("arm_state_topic", arm_state_topic, "/arm_state");

  std::string object_pose_topic;
  nh_.param<std::string>("object_pose_topic", object_pose_topic,
                         "/object_pose");

  std::string object_state_topic;
  nh_.param<std::string>("object_state_topic", object_state_topic,
                         "/object_state");

  std::string table_state_topic;
  nh_.param<std::string>("table_state_topic", table_state_topic,
                         "/table/joint_state");

  nh_.param<bool>("exact_sync", exact_sync_, false);
  
  nh_.param<double>("table_x",table_x,0);
  nh_.param<double>("table_y",table_y,0);
  nh_.param<double>("table_z",table_z,0);

  // normal subscribers
  arm_state_subscriber_ = nh_.subscribe(
      arm_state_topic, 1, &StateObserver::arm_state_callback, this);
  object_state_subscriber_ = nh_.subscribe(
      object_state_topic, 1, &StateObserver::object_state_callback, this);

  // TOODO(giuseppe+boyang) we commented this out in last test
  // {
  //   approx_message_filter_ =
  //       std::make_unique<message_filters::Synchronizer<ApproximatePolicy>>(
  //           ApproximatePolicy(1), arm_sub_, obj_state_sub_);
  //   approx_message_filter_->registerCallback(
  //       boost::bind(&StateObserver::message_filter_cb, this, _1, _2));
  // }
  //}

  // ros publishing
  state_publisher_ =
      nh_.advertise<manipulation_msgs::State>("/observer/state", 10);
  object_state_publisher_ = nh_.advertise<sensor_msgs::JointState>(
      "/observer/object/joint_state", 10);

  // init object state
  object_state_.name.push_back("object");
  for (int i = 0; i < 7; i++) {
    object_state_.position.push_back(0.0);
    object_state_.velocity.push_back(0.0);
  }

  // TODO : this default obj pos is hardcode now, and does not always equal to the init state of dynamics
  object_state_.position[0] = 1.2;
  object_state_.position[1] = 0;
  object_state_.position[2] = 0.15;
  object_state_.position[6] = 1.0;  // quat w = 1
  // TODO:(Boyang)  currently use velocity to pass geometry value
  object_state_.velocity[4] = 0.5;  // set the geometry to a non-zero default value
  object_state_.velocity[5] = 0.5; 

  articulation_first_computation_ = true;
  ROS_INFO("State observer inited");

}

bool StateObserver::init_ros() {
  // table
  table_state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("/table/joint_state", 10);
  table_trans.header.frame_id = "world";
  table_trans.child_frame_id = "table_frame";
  // object
  object_state_trans.header.frame_id = "world";
  object_state_trans.child_frame_id = "mug_frame";

  return true;
}
bool StateObserver::initialize() {
  if (!init_ros()) {
    ROS_INFO("Failed to init ros in state observer");
    return false;
  }
  return true;
}

void StateObserver::publish_state() {
  manipulation::conversions::toMsg_panda(time_, q_, dq_, object_state_, false,
                                         state_ros_);

  table_state_.header.stamp = ros::Time::now();
  table_trans.header.stamp = ros::Time::now();
  table_trans.transform.translation.x = table_x;  // TODO (boyang): table state is hardcoded, good for now
  table_trans.transform.translation.y = table_y;
  table_trans.transform.translation.z = table_z;
  tf2::Quaternion q_table;
  q_table.setRPY(0, 0, 0);
  table_trans.transform.rotation.x = q_table.x();
  table_trans.transform.rotation.y = q_table.y();
  table_trans.transform.rotation.z = q_table.z();
  table_trans.transform.rotation.w = q_table.w();

  object_state_trans.header.stamp = ros::Time::now();
  // object_state_trans.transform.translation.x = object_state_.position[0];
  // object_state_trans.transform.translation.y = object_state_.position[1];
  // object_state_trans.transform.translation.z = object_state_.position[2];
  // object_state_trans.transform.rotation.x = object_state_.position[4];
  // object_state_trans.transform.rotation.y = object_state_.position[5];
  // object_state_trans.transform.rotation.z = object_state_.position[6];
  // object_state_trans.transform.rotation.w = object_state_.position[3];

  table_state_publisher_.publish(table_state_);
  broadcaster.sendTransform(table_trans);
  // broadcaster.sendTransform(object_state_trans);
  state_publisher_.publish(state_ros_);
}

void StateObserver::message_filter_cb(
    const sensor_msgs::JointStateConstPtr& arm_state,
    const manipulation_msgs::MugPrimitiveConstPtr& object_state ) {
  if (arm_state->header.stamp.toSec() <= previous_publishing_time_) {
    return;
  }
  arm_state_callback(arm_state);
  object_state_callback(object_state);
  publish_state();
}

void StateObserver::arm_state_callback(
    const sensor_msgs::JointStateConstPtr& msg) {
  time_ = msg->header.stamp.toSec();
  for (size_t i = 0; i < 9; i++) {
    q_(i) = msg->position[i];
    dq_(i) = msg->velocity[i];
  }
  previous_publishing_time_ = time_;
}

void StateObserver::object_state_callback(
   const manipulation_msgs::MugPrimitiveConstPtr& msg) {

  geometry_msgs::TransformStamped transform;

  geometry_msgs::PoseStamped pri_pos_in_pri;
  pri_pos_in_pri.header = msg->header;
  pri_pos_in_pri.pose.position = msg->pose.position;
  pri_pos_in_pri.pose.orientation = msg->pose.orientation;

  geometry_msgs::PoseStamped pri_pos_in_world;

  transform = tf_buffer_.lookupTransform( "world", msg->header.frame_id, ros::Time(0), ros::Duration(1.0));

  tf2::doTransform(pri_pos_in_pri, pri_pos_in_world, transform); 

  object_state_.header.stamp = msg->header.stamp;
  object_state_.header.frame_id = "world";

  object_state_.position[0] = pri_pos_in_world.pose.position.x;
  object_state_.position[1] = pri_pos_in_world.pose.position.y;
  object_state_.position[2] = pri_pos_in_world.pose.position.z;

  object_state_.position[3] = pri_pos_in_world.pose.orientation.x;
  object_state_.position[4] = pri_pos_in_world.pose.orientation.y;
  object_state_.position[5] = pri_pos_in_world.pose.orientation.z;
  object_state_.position[6] = pri_pos_in_world.pose.orientation.w;

  // TODO:(Boyang)  currently use velocity to pass geometry value
  object_state_.velocity[4] = msg->body_radius;
  object_state_.velocity[5] = msg->body_height;

  object_state_publisher_.publish(object_state_);
}

}
