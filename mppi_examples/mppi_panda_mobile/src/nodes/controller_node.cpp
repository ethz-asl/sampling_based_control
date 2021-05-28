/*!
 * @file     panda_control.cpp
 * @author   Giuseppe Rizzi
 * @date     11.06.2020
 * @version  1.0
 * @brief    description
 */

#include <map>
#include <rclcpp/rclcpp.hpp>
#include "mppi_panda_mobile/controller_interface.h"
#include "mppi_panda_mobile/dynamics.h"

using namespace panda_mobile;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("controller_node");

  // state and input for controller
  // hack: hard coded additional finger dimensions
  mppi::DynamicsBase::observation_t x((int)PandaMobileDim::STATE_DIMENSION + 2);
  mppi::DynamicsBase::input_t u((int)PandaMobileDim::INPUT_DIMENSION);
  x.setZero();
  u.setZero();

  // timing
  double start_time = node->get_clock()->now().seconds();
  double previous_time = 0.0;

  // robot description
  std::string robot_description;
  node->declare_parameter<std::string>("robot_description", "");
  node->get_parameter<std::string>("robot_description", robot_description);

  // controller
  auto controller = PandaMobileControllerInterface(node);

  // state update
  std::mutex state_mutex;
  auto state_callback = [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::unique_lock<std::mutex> lock(state_mutex);
    for (int i = 0; i < PandaMobileDim::STATE_DIMENSION; i++) {
      x(i) = msg->position[i];
    }
  };
  auto state_subscriber_ =
      node->create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states", 10, state_callback);

  sensor_msgs::msg::JointState joint_cmd;
  joint_cmd.name = {"x_velocity_joint", "y_velocity_joint", "w_velocity_joint"};
  for (int i = 1; i < 8; i++)
    joint_cmd.name.push_back("panda_joint" + std::to_string(i));

  // command
  joint_cmd.velocity.resize(PandaMobileDim::STATE_DIMENSION, 0.0);
  auto joint_cmd_publisher =
      node->create_publisher<sensor_msgs::msg::JointState>("/velocity_cmd", 1);

  // end effector publisher
  auto ee_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/end_effector", 10);
  geometry_msgs::msg::PoseStamped ee_pose;

  // init the controller
  bool ok = controller.init();
  if (!ok) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize controller!");
  } else {
    RCLCPP_INFO(node->get_logger(), "Controller initialized.");
  }

  // set first observation
  {
    std::unique_lock<std::mutex> lock(state_mutex);
    controller.set_observation(x, 0.0);
  }

  // start controller threads
  controller.start();
  rclcpp::Rate rate(100.0);

  while (rclcpp::ok()) {
    // avoid going back in the past
    double current_time = node->get_clock()->now().seconds() - start_time;
    previous_time =
       (current_time < previous_time) ? previous_time : current_time;

    {
      std::unique_lock<std::mutex> lock(state_mutex);
      controller.set_observation(x, previous_time);
      controller.get_input(x, u, previous_time);
    }

    //send command
    for (int i = 0; i < joint_cmd.velocity.size(); i++)
      joint_cmd.velocity[i] = u(i);
    joint_cmd_publisher->publish(joint_cmd);

    // debug
    {
      std::unique_lock<std::mutex> lock(state_mutex);
      ee_pose = controller.get_pose_end_effector_ros(x);
    }
    ee_publisher->publish(ee_pose);

    // process callback (update new state)
    rclcpp::spin_some(node);
    rate.sleep();
  }

  return 0;
}