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

  nh_.param<bool>("exact_sync", exact_sync_, false);

  // subscribers for message filter
  arm_sub_.subscribe(nh_, arm_state_topic, 10);
  object_sub_.subscribe(nh_, object_pose_topic, 10);
  obj_state_sub_.subscribe(nh_, object_state_topic, 10);

  // message filter with sync policy (timestamps must be the same -> only
  // possible in simulation) or approximate timestamp matching policy
  // (see http://wiki.ros.org/message_filters/ApproximateTime)

  // in real experiment we estimate the joint values
  // this is only implemented for the shelf, would not work for other objects
  // Estimation is done subscribing to the handle odometry from vicon
  // In simulation instead, we directly subscribe to the object state published
  // by the simulator

  // Not using sim, or exact sync policy for now
  //if (!simulation_) 
  //{
    // if (exact_sync_) {
    //   exact_message_filter_ =
    //       std::make_unique<message_filters::Synchronizer<ExactPolicy>>(
    //           ExactPolicy(10), arm_sub_, base_pose_sub_, base_twist_sub_,
    //           object_sub_, wrench_sub_);
    //   exact_message_filter_->registerCallback(boost::bind(
    //       &StateObserver::message_filter_cb, this, _1, _2, _3, _4, _5));
    // } 
    // else 
    {
      approx_message_filter_ =
          std::make_unique<message_filters::Synchronizer<ApproximatePolicy>>(
              ApproximatePolicy(10), arm_sub_, object_sub_);
      approx_message_filter_->registerCallback(boost::bind(
          &StateObserver::message_filter_cb, this, _1, _2));
    }
  //} 
  // else 
  // {
  //   if (exact_sync_) {
  //     exact_message_filter_sim_ =
  //         std::make_unique<message_filters::Synchronizer<ExactPolicySim>>(
  //             ExactPolicySim(10), arm_sub_, base_pose_sub_, base_twist_sub_,
  //             obj_state_sub_, wrench_sub_);
  //     exact_message_filter_sim_->registerCallback(boost::bind(
  //         &StateObserver::message_filter_cb_sim, this, _1, _2, _3, _4, _5));
  //   } else {
  //     approx_message_filter_sim_ =
  //         std::make_unique<message_filters::Synchronizer<ApproximatePolicySim>>(
  //             ApproximatePolicySim(10), arm_sub_, base_pose_sub_,
  //             base_twist_sub_, obj_state_sub_, wrench_sub_);
  //     approx_message_filter_sim_->registerCallback(boost::bind(
  //         &StateObserver::message_filter_cb_sim, this, _1, _2, _3, _4, _5));
  //   }
  // }

  // ros publishing
  state_publisher_ =
      nh_.advertise<manipulation_msgs::State>("/observer/state", 10);
  object_state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("/observer/object/joint_state", 10);
  // base_pose_publisher_ =
  //     nh_.advertise<geometry_msgs::PoseStamped>("/observer/base_pose", 1);
  // base_twist_publisher_ =
  //     nh_.advertise<geometry_msgs::TwistStamped>("/observer/base_twist", 1);
  // robot_state_publisher_ =
  //     nh_.advertise<sensor_msgs::JointState>("/observer/base/joint_state", 1);

  object_state_.name.push_back("articulation_joint");

  for(int i = 0 ; i < 7; i ++)
  {
    object_state_.position.push_back(0.0);
    object_state_.velocity.push_back(0.0);
  }

  articulation_first_computation_ = true;

  // robot_state_.name = {"x_base_joint", "y_base_joint", "pivot_joint"};
  // robot_state_.position.resize(robot_state_.name.size());
  // robot_state_.velocity.resize(robot_state_.name.size());
  // robot_state_.header.frame_id = "world";

  //base_twist_ros_.header.frame_id = "world";

  ext_tau_.setZero();
}

bool StateObserver::initialize() {
  // if (!nh_.getParam("base_alpha", base_alpha_) || base_alpha_ < 0 ||
  //     base_alpha_ > 1) {
  //   ROS_ERROR("Failed to parse base_alpha param or invalid.");
  //   return false;
  // }

  //if (!simulation_) 
  {
    // KDL::Tree object_kinematics;
    // if (!kdl_parser::treeFromParam("object_description", object_kinematics)) {
    //   ROS_ERROR("Failed to create KDL::Tree from 'object_description'");
    //   return false;
    // }

    // KDL::Chain chain;
    // object_kinematics.getChain("shelf", "handle_link", chain);
    // if (chain.getNrOfJoints() > 1) {
    //   ROS_ERROR(
    //       "The object has more then one joint. Only one joint supported!");
    //   return false;
    // }
    // if (chain.getNrOfJoints() == 0) {
    //   ROS_ERROR("Failed to parse the object kinematic chain.");
    //   return false;
    // }

    // // at start-up door is closed
    // KDL::JntArray joint_pos(chain.getNrOfJoints());

    // // required to calibrate the initial shelf position
    // KDL::Frame T_shelf_handle_KDL;
    // KDL::ChainFkSolverPos_recursive fk_solver_shelf(chain);
    // fk_solver_shelf.JntToCart(joint_pos, T_shelf_handle_KDL);
    // tf::transformKDLToEigen(T_shelf_handle_KDL.Inverse(), T_handle0_shelf_);

    // // required to now the origin hinge position
    // KDL::Frame T_door_handle_KDL;
    // object_kinematics.getChain("axis_link", "handle_link", chain);
    // KDL::ChainFkSolverPos_recursive fk_solver_hinge(chain);
    // fk_solver_hinge.JntToCart(joint_pos, T_door_handle_KDL);
    // tf::transformKDLToEigen(T_door_handle_KDL.Inverse(), T_handle0_hinge_);
  }

  // get the relative pose of base to reference frame
  // KDL::Tree robot_kinematics;
  // if (!kdl_parser::treeFromParam("robot_description", robot_kinematics)) {
  //   ROS_ERROR("Failed to create KDL::Tree from 'robot_description'");
  //   return false;
  // }

  // KDL::Chain robot_chain;
  // if (!robot_kinematics.getChain("base_link", "reference_link", robot_chain)) {
  //   ROS_ERROR("Failed to extract chain from base_link to reference_link");
  //   return false;
  // }

  // if (!robot_kinematics.getChain("world", "panda_hand", world_to_ee_chain_)) {
  //   ROS_ERROR("Failed to extract chain from world to panda_hand");
  //   return false;
  // }
  // kdl_joints_.resize(world_to_ee_chain_.getNrOfJoints());
  // J_world_ee_.resize(world_to_ee_chain_.getNrOfJoints());
  // jacobian_solver_ =
  //     std::make_unique<KDL::ChainJntToJacSolver>(world_to_ee_chain_);

  // KDL::Frame T_base_reference_KDL;
  // KDL::JntArray robot_joint_pos(robot_chain.getNrOfJoints());
  // KDL::ChainFkSolverPos_recursive robot_solver(robot_chain);
  // robot_solver.JntToCart(robot_joint_pos, T_base_reference_KDL);
  // tf::transformKDLToEigen(T_base_reference_KDL.Inverse(), T_reference_base_);

  // ROS_INFO_STREAM("Static transformations summary: "
  //                 << std::endl
  //                 << " T_shelf_handle:\n "
  //                 << T_handle0_shelf_.inverse().matrix() << std::endl
  //                 << " T_hinge_handle:\n "
  //                 << T_handle0_hinge_.inverse().matrix() << std::endl
  //                 << " T_base_reference:\n "
  //                 << T_reference_base_.inverse().matrix() << std::endl);
  // ROS_INFO("Robot observer correctly initialized.");

  return true;
}

void StateObserver::message_filter_cb(
    const sensor_msgs::JointStateConstPtr& arm_state,
    const sensor_msgs::JointStateConstPtr& object_state) {
  if (arm_state->header.stamp.toSec() <= previous_publishing_time_) return;

  arm_state_callback(arm_state);
  object_state_callback(object_state);
  // base_pose_callback(base_pose);
  // base_twist_callback(base_twist);
  //object_pose_callback(object_odom);
  // this must be executed last as it requires computation from previous cb
  //wrench_callback(wrench);
  manipulation::conversions::toMsg_panda(
      time_,  q_, dq_, object_state_, false, state_ros_);

  state_publisher_.publish(state_ros_);
}


void StateObserver::arm_state_callback(
    const sensor_msgs::JointStateConstPtr& msg) {
  if (!are_equal((int)(9), (int)msg->name.size(), (int)msg->position.size(),
                 (int)msg->velocity.size())) {
    ROS_WARN_STREAM_THROTTLE(
        2.0, "Joint state fields have the wrong size." << msg->name.size());
    return;
  }
  time_ = msg->header.stamp.toSec();
  for (size_t i = 0; i < 9; i++) {
    q_(i) = msg->position[i];
    dq_(i) = msg->velocity[i];
  }
  previous_publishing_time_ = time_;
}

void StateObserver::object_state_callback(
    const sensor_msgs::JointStateConstPtr& msg) {
  object_state_.header.stamp = msg->header.stamp;
  for(int i = 0; i < 7; i ++)
  {
    object_state_.position[i] = msg->position[i];
    object_state_.velocity[i] = msg->velocity[i];
  }
  object_state_publisher_.publish(object_state_);
}

void StateObserver::object_pose_callback(
    const nav_msgs::OdometryConstPtr& msg) {
  std::cout << "In object pose callback" << std::endl;
  if (simulation_) return;
  tf::poseMsgToEigen(msg->pose.pose, T_world_handle_);
  if (articulation_first_computation_) {
    ROS_INFO("First computation of the shelf pose.");
    T_world_shelf_ = T_world_handle_ * T_handle0_shelf_;
    T_hinge_world_ = (T_world_handle_ * T_handle0_hinge_).inverse();
    T_hinge_handle_init_ = T_hinge_world_ * T_world_handle_;

    start_relative_angle_ = std::atan2(T_hinge_handle_init_.translation().x(),
                                       T_hinge_handle_init_.translation().y());
    articulation_first_computation_ = false;

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

  double velocity_norm =
      std::sqrt(msg->twist.twist.linear.x * msg->twist.twist.linear.x +
                msg->twist.twist.linear.y + msg->twist.twist.linear.y);
  double theta_dot = velocity_norm / T_hinge_handle_.translation().x();

  object_state_.header.stamp = msg->header.stamp;
  object_state_.position[0] = theta_new;
  object_state_.velocity[0] = theta_dot;
  object_state_publisher_.publish(object_state_);
}


// void StateObserver::message_filter_cb_sim(
//     const sensor_msgs::JointStateConstPtr& arm_state,
//     const nav_msgs::OdometryConstPtr& base_pose,
//     const nav_msgs::OdometryConstPtr& base_twist,
//     const sensor_msgs::JointStateConstPtr& object_state,
//     const geometry_msgs::WrenchStampedConstPtr& wrench) {
//   if (arm_state->header.stamp.toSec() <= previous_publishing_time_) return;

//   arm_state_callback(arm_state);
//   // base_pose_callback(base_pose);
//   // base_twist_callback(base_twist);
//   object_state_callback(object_state);

//   // this must be executed last as it requires computation from previous cb
//   wrench_callback(wrench);

//   manipulation::conversions::toMsg(
//       time_, base_pose_, base_twist_, ext_tau_.head<3>(), q_, dq_,
//       ext_tau_.tail<9>(), object_state_.position[0], object_state_.velocity[0],
//       contact_state_, tank_state_, state_ros_);

//   state_publisher_.publish(state_ros_);
// }

// void StateObserver::base_pose_callback(const nav_msgs::OdometryConstPtr& msg) {
//   tf::poseMsgToEigen(msg->pose.pose, T_world_reference_);
//   T_world_base_ = T_world_reference_ * T_reference_base_;

//   base_pose_.x() = T_world_base_.translation().x();
//   base_pose_.y() = T_world_base_.translation().y();

//   // 2d projection of forward motion axis
//   Eigen::Vector3d ix = T_world_base_.rotation().col(0);
//   base_pose_.z() = std::atan2(ix.y(), ix.x());

//   // publish to ros
//   geometry_msgs::PoseStamped base_pose;
//   base_pose.header.stamp = ros::Time::now();
//   base_pose.header.frame_id = "world";
//   base_pose.pose.position.x = base_pose_.x();
//   base_pose.pose.position.y = base_pose_.y();
//   base_pose.pose.position.z = 0.0;
//   Eigen::Quaterniond q(
//       Eigen::AngleAxisd(base_pose_.z(), Eigen::Vector3d::UnitZ()));
//   base_pose.pose.orientation.x = q.x();
//   base_pose.pose.orientation.y = q.y();
//   base_pose.pose.orientation.z = q.z();
//   base_pose.pose.orientation.w = q.w();
//   base_pose_publisher_.publish(base_pose);

//   robot_state_.header.stamp = msg->header.stamp;
//   robot_state_.position[0] = base_pose_.x();
//   robot_state_.position[1] = base_pose_.y();
//   robot_state_.position[2] = base_pose_.z();
//   robot_state_publisher_.publish(robot_state_);
// }

// void StateObserver::base_twist_callback(const nav_msgs::OdometryConstPtr& msg) {
//   Eigen::Vector3d odom_base_twist(msg->twist.twist.linear.x,
//                                   msg->twist.twist.linear.y,
//                                   msg->twist.twist.angular.z);
//   odom_base_twist = Eigen::AngleAxis(base_pose_.z(), Eigen::Vector3d::UnitZ()) *
//                     odom_base_twist;
//   base_twist_ = base_alpha_ * base_twist_ + (1 - base_alpha_) * odom_base_twist;
//   base_twist_ros_.header.stamp = msg->header.stamp;
//   base_twist_ros_.twist.linear.x = base_twist_.x();
//   base_twist_ros_.twist.linear.y = base_twist_.y();
//   base_twist_ros_.twist.angular.z = base_twist_.z();
//   base_twist_publisher_.publish(base_twist_ros_);
// }

// void StateObserver::wrench_callback(
//     const geometry_msgs::WrenchStampedConstPtr& msg) {
//   // wrench is in the end effector frame. Need to convert to world frame
//   Eigen::Affine3d T_world_sensor;
//   geometry_msgs::TransformStamped transform;

//   try {
//     transform = tf_buffer_.lookupTransform("world", msg->header.frame_id,
//                                            ros::Time(0), ros::Duration(1.0));
//   } catch (tf2::TransformException& ex) {
//     ROS_WARN("%s", ex.what());
//     return;
//   }

//   tf::transformMsgToEigen(transform.transform, T_world_sensor);

//   // update kdl joints with the latest measurements
//   kdl_joints_(0) = base_pose_.x();
//   kdl_joints_(1) = base_pose_.y();
//   kdl_joints_(2) = base_pose_.z();
//   for (int i = 0; i < 7; i++) {
//     kdl_joints_(3 + i) = q_(i);
//   }

//   jacobian_solver_->JntToJac(kdl_joints_, J_world_ee_);

//   // filter wrench measurements
//   const static double alpha = 0.1;
//   wrench_eigen_(0) =
//       alpha * wrench_eigen_(0) + (1.0 - alpha) * msg->wrench.force.x;
//   wrench_eigen_(1) =
//       alpha * wrench_eigen_(1) + (1.0 - alpha) * msg->wrench.force.y;
//   wrench_eigen_(2) =
//       alpha * wrench_eigen_(2) + (1.0 - alpha) * msg->wrench.force.z;
//   wrench_eigen_(3) =
//       alpha * wrench_eigen_(3) + (1.0 - alpha) * msg->wrench.torque.x;
//   wrench_eigen_(4) =
//       alpha * wrench_eigen_(4) + (1.0 - alpha) * msg->wrench.torque.y;
//   wrench_eigen_(5) =
//       alpha * wrench_eigen_(5) + (1.0 - alpha) * msg->wrench.torque.z;
//   wrench_eigen_.head<3>() = T_world_sensor.rotation() * wrench_eigen_.head<3>();
//   wrench_eigen_.tail<3>() = T_world_sensor.rotation() * wrench_eigen_.tail<3>();

//   // detect contact from wrench using small threshold
//   contact_state_ = wrench_eigen_.norm() > 0.1;
  
//   // clang-format off
//   J_world_ee_.data.topLeftCorner<3, 3>() << std::cos(base_pose_.z()), std::sin(base_pose_.z()), 0,
//                                             -std::sin(base_pose_.z()), std::cos(base_pose_.z()), 0,
//                                             0, 0, 1;
//   // clang-format on
//   ext_tau_ = J_world_ee_.data.transpose() * wrench_eigen_;

//   // sanitize torque
//   for (int i = 0; i < ext_tau_.size(); i++) {
//     ext_tau_(i) = std::abs(ext_tau_(i)) > 1e3 ? 0.0 : ext_tau_(i);
//   }
// }

}  // namespace manipulation_panda
