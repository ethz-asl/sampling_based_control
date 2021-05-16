/*!
 * @file     controller_interface.h
 * @author   Giuseppe Rizzi
 * @date     08.09.2020
 * @version  1.0
 * @brief    description
 */
#pragma once
#include <mppi_pinocchio/model.h>
#include <mppi_ros/controller_interface.h>

#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace panda_mobile {

class PandaMobileControllerInterface : public mppi_ros::ControllerRos {
 public:
  explicit PandaMobileControllerInterface(rclcpp::Node::SharedPtr& node)
      : ControllerRos(node){};
  ~PandaMobileControllerInterface() = default;

  bool init_ros() override;
  void publish_ros() override;
  bool update_reference() override;

  mppi_pinocchio::Pose get_pose_end_effector(const mppi::observation_t& x);
  geometry_msgs::msg::PoseStamped get_pose_end_effector_ros(
      const mppi::observation_t& x);

 private:
  void init_model(const std::string& robot_description);
  bool set_controller(std::shared_ptr<mppi::PathIntegral>& controller) override;

  void ee_pose_desired_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void obstacle_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

 public:
  mppi::SolverConfig config_;

 private:
  mppi::input_array_t u_opt_;
  mppi::observation_array_t x_opt_;

  std::mutex reference_mutex_;
  mppi::reference_trajectory_t ref_;

  double obstacle_radius_;
  mppi_pinocchio::RobotModel robot_model_;

  // ros
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimal_trajectory_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_marker_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr obstacle_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_desired_subscriber_;

  nav_msgs::msg::Path optimal_path_;
  geometry_msgs::msg::PoseStamped obstacle_pose_;
  geometry_msgs::msg::PoseStamped ee_desired_pose_;
  visualization_msgs::msg::Marker obstacle_marker_;
};

}  // namespace panda_mobile