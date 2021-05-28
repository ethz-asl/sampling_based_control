/*!
 * @file     panda_control.cpp
 * @author   Giuseppe Rizzi
 * @date     11.06.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_panda_mobile/controller_interface.h"
#include "mppi_panda_mobile/dynamics.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/node.hpp>

using namespace panda_mobile;

int main(int argc, char** argv) {
  // ros interface
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("panda_mobile_control");

  bool sequential;
  node->declare_parameter<bool>("sequential", false);
  node->get_parameter<bool>("sequential", sequential);

  double max_sim_time;
  node->declare_parameter<double>("max_sim_time", 0.0);
  node->get_parameter<double>("max_sim_time", max_sim_time);

  std::vector<double> initial_configuration;
  node->declare_parameter<std::vector<double>>("initial_configuration", {});
  node->get_parameter<std::vector<double>>("initial_configuration",
                                           initial_configuration);

  std::string robot_description;
  node->declare_parameter<std::string>("robot_description", "");
  node->get_parameter<std::string>("robot_description", robot_description);

  bool static_optimization;
  node->declare_parameter<bool>("static_optimization", false);
  node->get_parameter<bool>("static_optimization", static_optimization);

  double sim_dt;
  node->declare_parameter<double>("sim_dt", 0.01);
  node->get_parameter<double>("sim_dt", sim_dt);

  auto controller = PandaMobileControllerInterface(node);
  auto simulation = PandaMobileDynamics(robot_description);

  Eigen::VectorXd x = Eigen::VectorXd::Zero(PandaMobileDim::STATE_DIMENSION);
  for (int i = 0; i < initial_configuration.size(); i++)
    x(i) = initial_configuration[i];
  simulation.reset(x);

  mppi::DynamicsBase::input_t u;
  u = simulation.get_zero_input(x);

  // joint state publisher
  auto state_publisher =
      node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  sensor_msgs::msg::JointState joint_state;
  joint_state.name = {"x_velocity_joint", "y_velocity_joint",
                      "w_velocity_joint", "panda_joint1",
                      "panda_joint2",     "panda_joint3",
                      "panda_joint4",     "panda_joint5",
                      "panda_joint6",     "panda_joint7"};
  joint_state.position.resize(joint_state.name.size());
  joint_state.header.frame_id = "base";

  auto ee_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/end_effector", 10);
  geometry_msgs::msg::PoseStamped ee_pose;

  // init the controller
  double sim_time = 0.0;
  bool ok = controller.init();
  if (!ok) {
    throw std::runtime_error("Failed to initialzied controller!");
  }
  RCLCPP_INFO(node->get_logger(), "Started controller.");

  // set the very first observation
  controller.set_observation(x, sim_time);

  if (!sequential) controller.start();

  while (rclcpp::ok()) {
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
    for (int i = 0; i < joint_state.position.size(); i++)
      joint_state.position[i] = x(i);
    joint_state.header.stamp = node->now();
    state_publisher->publish(joint_state);

    if (!static_optimization) {
      x = simulation.step(u, sim_dt);
      sim_time += sim_dt;
    }

    ee_pose = controller.get_pose_end_effector_ros(x);
    ee_publisher->publish(ee_pose);

    auto end = std::chrono::steady_clock::now();
    auto elapsed_s =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count() /
        1000.0;
    double delta = sim_dt - elapsed_s;
     if (delta > 0){
       rclcpp::sleep_for(std::chrono::nanoseconds(int(delta*1e9)));
     }

    if (max_sim_time > 0 && sim_time > max_sim_time) {
      RCLCPP_INFO_STREAM(
          node->get_logger(),
          "Reached maximum sim time: " << max_sim_time << "s. Exiting.");
      break;
    }
    rclcpp::spin_some(node);
  }

  return 0;
}