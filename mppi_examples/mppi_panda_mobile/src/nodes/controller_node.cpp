/*!
 * @file     panda_control.cpp
 * @author   Giuseppe Rizzi
 * @date     11.06.2020
 * @version  1.0
 * @brief    description
 */

#include <map>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "mppi_panda_mobile/controller_interface.h"
#include "mppi_panda_mobile/dynamics.h"
#include "geometry_msgs/msg/twist.hpp"

using namespace panda_mobile;
using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("controller_node");

  // state and input for controller
  // hack: hard coded additional finger dimensions
  mppi::DynamicsBase::observation_t x((int)PandaMobileDim::STATE_DIMENSION + 2);
  mppi::DynamicsBase::input_t u((int)PandaMobileDim::INPUT_DIMENSION);
  x.setZero();
  u.setZero();

  // robot description
  std::string robot_description;
  node->declare_parameter<std::string>("robot_description", "");
  node->get_parameter<std::string>("robot_description", robot_description);

  // controller
  auto controller = PandaMobileControllerInterface(node);

  // command
  geometry_msgs::msg::Twist base_cmd;
  sensor_msgs::msg::JointState joint_cmd;
  
  for (int i = 1; i < 8; i++){
    joint_cmd.name.push_back("panda_joint" + std::to_string(i));
    joint_cmd.position.push_back(0.0);
    joint_cmd.velocity.push_back(0.0);    
  }


  auto joint_cmd_publisher = node->create_publisher<JointState>("/velocity_cmd", 1);
  auto base_cmd_publisher = node->create_publisher<Twist>("/base_cmd_vel", 1);
  
  // state update
  std::mutex observation_mutex;
  double observation_time = 0.0;
  double last_command_time = 0.0;
  std::atomic_bool first_observation = true;

  auto state_callback = [&](const JointState::SharedPtr msg) {
    std::unique_lock<std::mutex> lock(observation_mutex);
    observation_time = rclcpp::Time(msg->header.stamp).seconds();
    for (int i = 0; i < PandaMobileDim::STATE_DIMENSION; i++) {
      x(i) = msg->position[i];
      if (first_observation && i>2) 
        joint_cmd.position[i-3] = x(i);
    }
    if (first_observation){
      last_command_time = observation_time;
      RCLCPP_INFO_STREAM(node->get_logger(), "Received first observation: " << x.transpose());
    }
    first_observation = false;
  };

  auto state_subscriber_ = node->create_subscription<JointState>("/joint_states", 10, state_callback);

  // end effector publisher
  auto ee_publisher = node->create_publisher<PoseStamped>("/end_effector", 10);
  PoseStamped ee_pose;

  // init the controller
  bool ok = controller.init();
  if (!ok) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize controller!");
  } else {
    RCLCPP_INFO(node->get_logger(), "Controller initialized.");
  }

  // set first observation
  {
    std::unique_lock<std::mutex> lock(observation_mutex);
    controller.set_observation(x, observation_time);
  }

  // start controller threads
  controller.start();
 
  std::array<double, 7> sign{1, -1, 1, 1, -1, 1, 1};
  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    {
      std::unique_lock<std::mutex> lock(observation_mutex);
      controller.set_observation(x, observation_time);
      controller.get_input(x, u, observation_time);
    }

    //send command
    base_cmd.linear.x = u(0);
    base_cmd.linear.y = u(1);
    base_cmd.angular.z = u(2);
    base_cmd_publisher->publish(base_cmd);
    if (!first_observation){
      for (int i = 0; i < joint_cmd.velocity.size(); i++){
        joint_cmd.position[i] += u(i + 3) * (observation_time - last_command_time) * sign[i];
        joint_cmd.velocity[i] = u(i + 3) * sign[i];
      }
      joint_cmd.header.stamp = rclcpp::Time((int64_t)(observation_time*1e9));
      joint_cmd_publisher->publish(joint_cmd);
    }
    last_command_time = observation_time;

    // debug
    {
      std::unique_lock<std::mutex> lock(observation_mutex);
      ee_pose = controller.get_pose_end_effector_ros(x);
    }
    ee_publisher->publish(ee_pose);

    rate.sleep();

    // process callback (update new state)
    rclcpp::spin_some(node);
  }

  return 0;
}