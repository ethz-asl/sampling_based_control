/*!
 * @file     controller_interface.h
 * @author   Giuseppe Rizzi
 * @date     08.09.2020
 * @version  1.0
 * @brief    description
 */
#pragma once
#include "mppi_manipulation/cost.h"
#include "mppi_manipulation/dynamics.h"
#include <mppi_ros/controller_interface.h>

#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int64.h>
#include <visualization_msgs/Marker.h>

namespace manipulation {

class PandaControllerInterface : public mppi_ros::ControllerRos {
 public:
  explicit PandaControllerInterface(ros::NodeHandle& nh) : ControllerRos(nh){};
  ~PandaControllerInterface() = default;

  bool init_ros() override;
  void publish_ros() override;
  bool update_reference() override;

  pinocchio::SE3 get_pose_handle(const mppi::observation_t& x);
  pinocchio::SE3 get_pose_end_effector(const mppi::observation_t& x);

  geometry_msgs::PoseStamped pose_pinocchio_to_ros(const pinocchio::SE3& pose);
  geometry_msgs::PoseStamped get_pose_handle_ros(const mppi::observation_t& x);
  geometry_msgs::PoseStamped get_pose_end_effector_ros(const mppi::observation_t& x);

 private:
  void init_model(const std::string& robot_description, const std::string& object_description);
  bool set_controller(std::shared_ptr<mppi::PathIntegral>& controller) override;

  void ee_pose_desired_callback(const geometry_msgs::PoseStampedConstPtr& msg);
  void obstacle_callback(const geometry_msgs::PoseStampedConstPtr& msg);
  void mode_callback(const std_msgs::Int64ConstPtr& msg);

 public:
  mppi::SolverConfig config_;

 private:
  bool fixed_base_;
  mppi::input_array_t u_opt_;
  mppi::observation_array_t x_opt_;

  size_t last_ee_ref_id_;
  size_t last_ob_ref_id_;
  std::mutex reference_mutex_;
  mppi::reference_trajectory_t ref_;

  // arm
  double obstacle_radius_;
  pinocchio::Data data_;
  pinocchio::Model model_;

  // door
  pinocchio::Model object_model_;
  pinocchio::Data object_data_;
  int handle_idx_;

  // ros
  ros::Publisher optimal_trajectory_publisher_;
  ros::Publisher obstacle_marker_publisher_;

  ros::Subscriber mode_subscriber_;
  ros::Subscriber obstacle_subscriber_;
  ros::Subscriber ee_pose_desired_subscriber_;

  nav_msgs::Path optimal_path_;
  geometry_msgs::PoseStamped obstacle_pose_;
  geometry_msgs::PoseStamped ee_desired_pose_;
  visualization_msgs::Marker obstacle_marker_;
};

}