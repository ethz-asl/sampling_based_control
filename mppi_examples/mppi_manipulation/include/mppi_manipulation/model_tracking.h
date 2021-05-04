//
// Created by giuseppe on 25.04.21.
//

#pragma once
#include <mppi_pinocchio/model.h>
#include <mppi_ros/controller_interface.h>
#include <mppi_tools/model_tracking_controller.h>

#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int64.h>
#include <visualization_msgs/Marker.h>

namespace manipulation {

class ManipulationTrackingController
    : public mppi_tools::ModelTrackingController {
 public:
  explicit ManipulationTrackingController(ros::NodeHandle& nh);
  ~ManipulationTrackingController() = default;

  bool init_ros();
  void publish_ros();

  mppi_pinocchio::Pose get_pose_handle(const mppi::observation_t& x);
  mppi_pinocchio::Pose get_pose_end_effector(const mppi::observation_t& x);

  geometry_msgs::PoseStamped get_pose_handle_ros(const mppi::observation_t& x);
  geometry_msgs::PoseStamped get_pose_end_effector_ros(
      const mppi::observation_t& x);
  geometry_msgs::PoseStamped get_pose_base(const mppi::observation_t& x);

 private:
  void init_model(const std::string& robot_description,
                  const std::string& object_description);

  // TODO(giuseppe) make virtual
  bool setup();

  void ee_pose_desired_callback(const geometry_msgs::PoseStampedConstPtr& msg);
  void mode_callback(const std_msgs::Int64ConstPtr& msg);

 public:
  bool fixed_base_;
  mppi::config_t config_;

 private:
  ros::NodeHandle nh_;

  mppi::input_array_t u_opt_;
  mppi::observation_array_t x_opt_;

  size_t last_ee_ref_id_;
  size_t last_ob_ref_id_;
  std::mutex reference_mutex_;
  mppi::reference_trajectory_t ref_;

  mppi_pinocchio::RobotModel robot_model_;
  mppi_pinocchio::RobotModel object_model_;

  // ros
  ros::Publisher optimal_trajectory_publisher_;
  ros::Publisher optimal_base_trajectory_publisher_;
  ros::Publisher obstacle_marker_publisher_;
  ros::Publisher base_twist_from_path_publisher_;
  ros::Publisher pose_handle_publisher_;

  ros::Subscriber mode_subscriber_;
  ros::Subscriber ee_pose_desired_subscriber_;

  nav_msgs::Path optimal_path_;
  nav_msgs::Path optimal_base_path_;
  geometry_msgs::PoseStamped obstacle_pose_;
  geometry_msgs::PoseStamped ee_desired_pose_;
  visualization_msgs::Marker obstacle_marker_;

  ros::Publisher state_publisher_;
  ros::Publisher object_state_publisher_;
  ros::Publisher contact_forces_publisher_;
  ros::Publisher ee_publisher_;
  ros::Publisher handle_publisher_;

  sensor_msgs::JointState joint_state_;
  sensor_msgs::JointState object_state_;
  visualization_msgs::Marker force_marker_;
};

}  // namespace manipulation