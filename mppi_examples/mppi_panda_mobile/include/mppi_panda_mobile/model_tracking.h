//
// Created by giuseppe on 25.04.21.
//

#pragma once

#include <mppi_pinocchio/model.h>
#include <mppi_tools/model_tracking_controller.h>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>

namespace panda_mobile {

class PandaMobileModelTracking {
 public:
  explicit PandaMobileModelTracking(ros::NodeHandle& nh): nh_(nh){};
  ~PandaMobileModelTracking() = default;

  bool init_ros();
  void publish_ros();
  bool update_reference();

  mppi_pinocchio::Pose get_pose_end_effector(const mppi::observation_t& x);
  geometry_msgs::PoseStamped get_pose_end_effector_ros(
      const mppi::observation_t& x);

 private:
  void init_model(const std::string& robot_description);
  bool set_controller(std::shared_ptr<mppi::PathIntegral>& controller);

  void ee_pose_desired_callback(const geometry_msgs::PoseStampedConstPtr& msg);
  void obstacle_callback(const geometry_msgs::PoseStampedConstPtr& msg);

 public:
  mppi::SolverConfig config_;
  // TODO(giuseppe) reorganize the publish ros into one function
  // void publish_state();

 private:
  // model tracking
  mppi_tools::ModelTrackingController model_tracking_;

  mppi::input_array_t u_opt_;
  mppi::observation_array_t x_opt_;

  size_t last_ee_ref_id_;
  size_t last_ob_ref_id_;
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

  nav_msgs::Path optimal_path_;
  geometry_msgs::PoseStamped obstacle_pose_;
  geometry_msgs::PoseStamped ee_desired_pose_;
  visualization_msgs::Marker obstacle_marker_;
};

}  // namespace panda_mobile