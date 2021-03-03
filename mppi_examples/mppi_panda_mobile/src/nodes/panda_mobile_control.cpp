/*!
 * @file     panda_control.cpp
 * @author   Giuseppe Rizzi
 * @date     11.06.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_panda_mobile/controller_interface.h"
#include "mppi_panda_mobile/dynamics.h"

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <chrono>

using namespace panda_mobile;

int main(int argc, char** argv) {
  // ros interface
  ros::init(argc, argv, "panda_mobile_control_node");
  ros::NodeHandle nh("~");

  auto sequential = nh.param<bool>("sequential", false);
  auto max_sim_time = nh.param<double>("max_sim_time", 0.0);

  auto controller = PandaMobileControllerInterface(nh);

  std::string robot_description =
      nh.param<std::string>("/robot_description", "");
  auto simulation = PandaMobileDynamics(robot_description);

  Eigen::VectorXd x = Eigen::VectorXd::Zero(PandaMobileDim::STATE_DIMENSION);
  auto initial_configuration =
      nh.param<std::vector<double>>("initial_configuration", {});
  for (size_t i = 0; i < initial_configuration.size(); i++)
    x(i) = initial_configuration[i];
  simulation.reset(x);

  mppi::DynamicsBase::input_t u;
  u = simulation.get_zero_input(x);

  // joint state publisher
  ros::Publisher state_publisher =
      nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
  sensor_msgs::JointState joint_state;
  joint_state.name = {"panda_joint1", "panda_joint2", "panda_joint3",
                      "panda_joint4", "panda_joint5", "panda_joint6",
                      "panda_joint7"};
  joint_state.position.resize(7);
  joint_state.header.frame_id = "base";

  // base tf
  tf2_ros::TransformBroadcaster tf_broadcaster;
  geometry_msgs::TransformStamped world_base_tf;
  world_base_tf.header.frame_id = "world";
  world_base_tf.child_frame_id = "base";

  ros::Publisher ee_publisher =
      nh.advertise<geometry_msgs::PoseStamped>("/end_effector", 10);
  geometry_msgs::PoseStamped ee_pose;

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

  if (!sequential) controller.start();

  while (ros::ok()) {
    auto start = std::chrono::steady_clock::now();

    controller.set_observation(x, sim_time);
    controller.get_input(x, u, sim_time);

    if (sequential) {
      controller.update_reference();
      controller.publish_ros_default();
      controller.publish_ros();
      controller.update_policy();
    }

    // publish joint state
    for (size_t i = 0; i < 7; i++) joint_state.position[i] = x(i);
    joint_state.header.stamp = ros::Time::now();
    state_publisher.publish(joint_state);

    if (!static_optimization) {
      x = simulation.step(u, sim_dt);
      sim_time += sim_dt;
    }

    // publish base transform
    world_base_tf.header.stamp = ros::Time::now();
    world_base_tf.transform.translation.x = x(7);
    world_base_tf.transform.translation.y = x(8);
    Eigen::Quaterniond q(Eigen::AngleAxisd(x(9), Eigen::Vector3d::UnitZ()));
    world_base_tf.transform.rotation.x = q.x();
    world_base_tf.transform.rotation.y = q.y();
    world_base_tf.transform.rotation.z = q.z();
    world_base_tf.transform.rotation.w = q.w();
    tf_broadcaster.sendTransform(world_base_tf);

    ee_pose = controller.get_pose_end_effector_ros(x);
    ee_publisher.publish(ee_pose);

    auto end = std::chrono::steady_clock::now();
    double elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count() /
        1000.0;
    if (sim_dt - elapsed > 0) ros::Duration(sim_dt - elapsed).sleep();

    if (max_sim_time > 0 and sim_time > max_sim_time) {
      ROS_INFO_STREAM("Reached maximum sim time: " << max_sim_time
                                                   << "s. Exiting.");
      break;
    }
    ros::spinOnce();
  }

  return 0;
}