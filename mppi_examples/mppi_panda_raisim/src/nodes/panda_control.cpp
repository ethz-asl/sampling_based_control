/*!
 * @file     panda_control.cpp
 * @author   Giuseppe Rizzi
 * @date     11.06.2020
 * @version  1.0
 * @brief    description
 */
#include "mppi_panda_raisim/controller_interface.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <chrono>

#include <visualization_msgs/MarkerArray.h>

using namespace panda;

int main(int argc, char** argv) {
  raisim::World::setActivationKey(
      "/home/giuseppe/git/raisimlib/rsc/activation.raisim");

  // ros interface
  ros::init(argc, argv, "panda_raisim_control_node");
  ros::NodeHandle nh("~");
  auto controller = PandaControllerInterface(nh);

  auto robot_description = nh.param<std::string>("/robot_description", "");
  auto robot_description_raisim =
      nh.param<std::string>("/robot_description_raisim", "");
  auto simulation =
      std::make_shared<PandaRaisimDynamics>(robot_description_raisim, 0.01);

  visualization_msgs::Marker force_marker;
  force_marker.type = visualization_msgs::Marker::ARROW;
  force_marker.header.frame_id = "world";
  force_marker.action = visualization_msgs::Marker::ADD;
  force_marker.pose.orientation.w = 1.0;
  force_marker.scale.x = 0.005;
  force_marker.scale.y = 0.01;
  force_marker.scale.z = 0.02;
  force_marker.color.r = 1.0;
  force_marker.color.b = 0.0;
  force_marker.color.g = 0.0;
  force_marker.color.a = 1.0;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(PandaDim::STATE_DIMENSION);
  Eigen::VectorXd x_nom = Eigen::VectorXd::Zero(PandaDim::STATE_DIMENSION);

  // set initial state (which is also equal to the one to be tracked)
  // the door position and velocity is already set to 0
  auto x0 = nh.param<std::vector<double>>("initial_configuration", {});
  for (size_t i = 0; i < x0.size(); i++) {
    x.head<PandaDim::JOINT_DIMENSION>()(i) = x0[i];
  }

  ROS_INFO_STREAM("Resetting initial state to " << x.transpose());
  simulation->reset(x);

  // init control input
  mppi::DynamicsBase::input_t u;
  u = simulation->get_zero_input(x);

  ros::Publisher ee_publisher =
      nh.advertise<geometry_msgs::PoseStamped>("/end_effector", 10);
  ros::Publisher handle_publisher =
      nh.advertise<geometry_msgs::PoseStamped>("/handle", 10);
  ros::Publisher ee_desired_publisher =
      nh.advertise<geometry_msgs::PoseStamped>("/ee_desired_nominal", 10);

  ros::Publisher state_publisher =
      nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
  ros::Publisher door_state_publisher =
      nh.advertise<sensor_msgs::JointState>("/door/joint_state", 10);
  ros::Publisher contact_forces_publisher =
      nh.advertise<visualization_msgs::MarkerArray>("/contact_forces", 10);

  sensor_msgs::JointState joint_state, door_state;
  joint_state.name = {
      "panda_joint1", "panda_joint2",        "panda_joint3",
      "panda_joint4", "panda_joint5",        "panda_joint6",
      "panda_joint7", "panda_finger_joint1", "panda_finger_joint2"};
  joint_state.position.resize(joint_state.name.size());
  joint_state.header.frame_id = "world";
  door_state.header.frame_id = "world";
  door_state.name = {"door_joint"};
  door_state.position.resize(1);

  geometry_msgs::PoseStamped ee_pose;
  geometry_msgs::PoseStamped handle_pose;
  geometry_msgs::PoseStamped ee_pose_desired;

  bool static_optimization = nh.param<bool>("static_optimization", false);
  double sim_dt = nh.param<double>("sim_dt", 0.01);

  double sim_time = 0.0;

  // init the controller
  bool ok = controller.init();
  if (!ok) {
    throw std::runtime_error("Failed to initialzied controller!");
  }

  // set the very first observation
  controller.set_observation(x, sim_time);

  // start controller
  // controller.start();

  bool freeze_robot = false;  // hack to freeze the robot once task is done
  while (ros::ok()) {
    auto start = std::chrono::steady_clock::now();

    controller.update_reference();
    controller.set_observation(x, sim_time);
    controller.update_policy();
    controller.get_input(x, u, sim_time);
    controller.publish_ros_default();
    controller.publish_ros();

    if (freeze_robot) u.setZero();

    if (!static_optimization) {
      x = simulation->step(u, sim_dt);
      sim_time += sim_dt;
    }

    for (size_t i = 0; i < PandaDim::JOINT_DIMENSION; i++)
      joint_state.position[i] = x(i);
    joint_state.header.stamp = ros::Time::now();
    state_publisher.publish(joint_state);

    door_state.header.stamp = ros::Time::now();
    door_state.position[0] = x(PandaDim::JOINT_DIMENSION * 2);
    door_state_publisher.publish(door_state);

    if (std::abs(door_state.position[0] - M_PI / 2.0) < 1.0 * M_PI / 180.0)
      freeze_robot = true;

    ee_pose = controller.get_pose_end_effector_ros(x);
    ee_publisher.publish(ee_pose);

    ee_pose_desired = controller.get_pose_end_effector_ros(x_nom);
    ee_desired_publisher.publish(ee_pose_desired);

    handle_pose = controller.get_pose_handle_ros(x);
    handle_publisher.publish(handle_pose);

    ROS_INFO_STREAM_THROTTLE(1.0, "In contact? " << x.tail<1>()(0));

    std::vector<force_t> forces = simulation->get_contact_forces();
    visualization_msgs::MarkerArray force_markers;
    for (const auto& force : forces) {
      force_marker.points.resize(2);
      force_marker.points[0].x = force.position(0);
      force_marker.points[0].y = force.position(1);
      force_marker.points[0].z = force.position(2);
      force_marker.points[1].x = force.position(0) + force.force(0) / 10.0;
      force_marker.points[1].y = force.position(1) + force.force(1) / 10.0;
      force_marker.points[1].z = force.position(2) + force.force(2) / 10.0;
      force_markers.markers.push_back(force_marker);
    }
    contact_forces_publisher.publish(force_markers);

    auto end = std::chrono::steady_clock::now();
    double elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count() /
        1000.0;
    if (sim_dt - elapsed > 0)
      ros::Duration(sim_dt - elapsed).sleep();
    else
      ROS_INFO_STREAM_THROTTLE(
          3.0, "Slower than real-time: " << elapsed / sim_dt << "x slower.");

    ros::spinOnce();
  }
}