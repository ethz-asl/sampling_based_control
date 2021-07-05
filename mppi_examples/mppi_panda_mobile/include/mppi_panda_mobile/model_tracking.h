//
// Created by giuseppe on 25.04.21.
//

#pragma once

#include <mppi_pinocchio/model.h>

#include <mppi/core/config.h>
#include <mppi/core/typedefs.h>

#include <mppi_tools/model_tracking_controller.h>

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

namespace panda_mobile {

class PandaMobileModelTracking : public mppi_tools::ModelTrackingController {
 public:
  explicit PandaMobileModelTracking(ros::NodeHandle& nh);
  ~PandaMobileModelTracking() = default;

  bool init_ros();
  bool setup();
  void publish_ros();

  mppi_pinocchio::Pose get_pose_end_effector(const mppi::observation_t& x);
  geometry_msgs::PoseStamped get_pose_end_effector_ros(
      const mppi::observation_t& x);

 private:

  void ee_pose_desired_callback(const geometry_msgs::PoseStampedConstPtr& msg);
  void obstacle_callback(const geometry_msgs::PoseStampedConstPtr& msg);

 public:
  bool initialized_;
  mppi::Config config_;

 private:
  mppi::input_array_t u_opt_;
  mppi::observation_array_t x_opt_;

  std::mutex reference_mutex_;
  mppi::reference_trajectory_t ref_;

  double obstacle_radius_;
  mppi_pinocchio::RobotModel robot_model_;

  // ros
  ros::NodeHandle nh_;
  ros::Publisher optimal_trajectory_publisher_;
  ros::Publisher obstacle_marker_publisher_;
  ros::Subscriber obstacle_subscriber_;
  ros::Subscriber ee_pose_desired_subscriber_;
  ros::Publisher state_publisher_;

  sensor_msgs::JointState joint_state_;
  nav_msgs::Path optimal_path_;
  geometry_msgs::PoseStamped obstacle_pose_;
  geometry_msgs::PoseStamped ee_desired_pose_;
  visualization_msgs::Marker obstacle_marker_;
};

}  // namespace panda_mobile