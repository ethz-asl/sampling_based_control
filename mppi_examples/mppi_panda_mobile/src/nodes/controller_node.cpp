/*!
 * @file     panda_control.cpp
 * @author   Giuseppe Rizzi
 * @date     11.06.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_panda_mobile/controller_interface.h"
#include <rclcpp/node.hpp>

using namespace panda_mobile;

int main(int argc, char** argv) {
  // ros interface
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("controller_node");

  std::string robot_description;
  node->declare_parameter<std::string>("robot_description", "");
  node->get_parameter<std::string>("robot_description", robot_description);

  auto controller = PandaMobileControllerInterface(node);

  Eigen::VectorXd x = Eigen::VectorXd::Zero(PandaMobileDim::STATE_DIMENSION);

  mppi::DynamicsBase::input_t u;
  u = simulation.get_zero_input(x);

  auto ee_publisher =
      node->create_publisher<geometry_msgs::msg::PoseStamped>("/end_effector", 10);
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

  controller.start();

  while (rclcpp::ok()) {
    auto start = std::chrono::steady_clock::now();

    controller.set_observation(x, sim_time);
    controller.get_input(x, u, sim_time);

    ee_pose = controller.get_pose_end_effector_ros(x);
    ee_publisher->publish(ee_pose);

    rclcpp::spin_some(node);
  }

  return 0;
}