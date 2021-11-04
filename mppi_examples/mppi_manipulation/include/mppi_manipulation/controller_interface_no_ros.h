/*!
 * @file     controller_interface_no_ros.h
 * @author   Giulio Schiavi
 * @date     04.11.21
 * @version  1.0
 * @brief    description
 */
#pragma once
#include <yaml-cpp/yaml.h>
#include <mppi_pinocchio/model.h>
#include <mppi_ros/controller_interface.h>

#include "mppi_manipulation/cost.h"
#include "mppi_manipulation/params/dynamics_params.h"
#include "mppi_manipulation/reference_scheduler.h"
#include "mppi_manipulation/config_no_ros.h"

#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int64.h>
#include <visualization_msgs/Marker.h>

namespace manipulation {

class PandaControllerInterfaceNoRos{
 public:
  PandaControllerInterfaceNoRos(const std::string& config_path);
  ~PandaControllerInterfaceNoRos() = default;



  bool init();
  void update_reference(const mppi::observation_t& x, const double t);

  mppi_pinocchio::Pose get_pose_handle(const mppi::observation_t& x);
  mppi_pinocchio::Pose get_pose_end_effector(const mppi::observation_t& x);

//  geometry_msgs::PoseStamped get_pose_handle_ros(const mppi::observation_t& x);
//  geometry_msgs::PoseStamped get_pose_end_effector_ros(
//      const mppi::observation_t& x);
//  geometry_msgs::PoseStamped get_pose_base(const mppi::observation_t& x);


  void set_mode(int mode);

  double object_tolerance_;

 private:
  double get_stage_cost(const mppi::observation_t& x, const mppi::input_t& u, const double t);
  bool init_reference_to_current_pose(const mppi::observation_t& x, const double t);
  inline std::shared_ptr<Solver>& get_controller() { return controller_; };
  void init_model(const std::string& robot_description, const std::string& object_description);

  bool set_controller(mppi::solver_ptr& controller);

  void ee_pose_desired_callback(const geometry_msgs::PoseStampedConstPtr& msg);
  void mode_callback(const std_msgs::Int64ConstPtr& msg);

  std::string config_path_;
  mppi::config_t config_;
  manipulation::Config manipulation_config_;
  bool reference_set_;
  std::unique_ptr<manipulation::PandaCost> local_cost_;

  mppi::input_array_t u_opt_;
  mppi::observation_array_t x_opt_;


  Eigen::VectorXd default_pose_;
  ReferenceScheduler reference_scheduler_;
  size_t last_ee_ref_id_;
  size_t last_ob_ref_id_;
  std::mutex reference_mutex_;
  mppi::reference_trajectory_t ref_;

  DynamicsParams dynamics_params_;
  mppi_pinocchio::RobotModel robot_model_;
  mppi_pinocchio::RobotModel object_model_;

  // from ControllerRos, former base class
  mppi::cost_ptr cost_;
  mppi::dynamics_ptr dynamics_;
//  mppi::config_t config_;

  std::shared_ptr<mppi::Solver> controller_ = nullptr;

  // ros
  ros::Subscriber mode_subscriber_;
  ros::Subscriber ee_pose_desired_subscriber_;

  nav_msgs::Path optimal_path_;
  nav_msgs::Path optimal_base_path_;
  geometry_msgs::PoseStamped obstacle_pose_;
  geometry_msgs::PoseStamped ee_desired_pose_;
  visualization_msgs::Marker obstacle_marker_;
};

}  // namespace manipulation