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

#include "mppi_manipulation/cost.h"
#include "mppi_manipulation/params/dynamics_params.h"
#include "mppi_manipulation/reference_scheduler.h"

#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int64.h>
#include <visualization_msgs/Marker.h>

namespace manipulation {

class PandaControllerInterface : public mppi_ros::ControllerRos {
 public:
  explicit PandaControllerInterface(ros::NodeHandle& nh) : ControllerRos(nh) {
    ROS_INFO("[PandaControllerInterface(ros::NodeHandle& nh)]");
  };
  ~PandaControllerInterface() = default;

  bool init_ros() override;
  void publish_ros() override;
  void update_reference(const mppi::observation_t& x, const double t);

  mppi_pinocchio::Pose get_pose_handle(const mppi::observation_t& x);
  mppi_pinocchio::Pose get_pose_end_effector(const mppi::observation_t& x);

  geometry_msgs::PoseStamped get_pose_handle_ros(const mppi::observation_t& x);
  geometry_msgs::PoseStamped get_pose_end_effector_ros(
      const mppi::observation_t& x);
  geometry_msgs::PoseStamped get_pose_base(const mppi::observation_t& x);
  double get_stage_cost(const mppi::observation_t& x, const mppi::input_t& u,
                        const double t);
  bool init_reference_to_current_pose(const mppi::observation_t& x,
                                      const double t);

 private:
  void init_model(const std::string& robot_description,
                  const std::string& object_description);

  bool set_controller(mppi::solver_ptr& controller) override;

  void ee_pose_desired_callback(const geometry_msgs::PoseStampedConstPtr& msg);
  void mode_callback(const std_msgs::Int64ConstPtr& msg);

 public:
  mppi::config_t config_;

 private:
  bool reference_set_;
  std::unique_ptr<manipulation::PandaCost> local_cost_;

  mppi::input_array_t u_opt_;
  mppi::observation_array_t x_opt_;

  double object_tolerance_;
  Eigen::VectorXd default_pose_;
  ReferenceScheduler reference_scheduler_;
  size_t last_ee_ref_id_;
  size_t last_ob_ref_id_;
  std::mutex reference_mutex_;
  mppi::reference_trajectory_t ref_;

  DynamicsParams dynamics_params_;
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
};

}  // namespace manipulation