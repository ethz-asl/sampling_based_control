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
  void set_observation(const mppi::observation_t &x, const double &t);
  bool update_policy();
  mppi::input_t get_input(const mppi::observation_t &x, const double &t);
  void set_reference(mppi::reference_array_t refs, mppi::time_array_t t);
  double get_stage_cost(const mppi::observation_t& x, const mppi::input_t& u, const double t);
  inline double get_rollout_cost(){return controller_->get_rollout_cost();};

  // not used
  void set_mode(int mode);
  void set_desired_ee_pose(const std::vector<double> pose);
  void update_reference(const mppi::observation_t& x, const double t);

 private:
  // functions
  bool init_ros();
  bool init_reference_to_current_pose(const mppi::observation_t& x, const double t);
  inline std::shared_ptr<Solver>& get_controller() { return controller_; };
  void init_model(const std::string& robot_description, const std::string& object_description);
  bool set_controller(mppi::solver_ptr& controller);


  // todo: GIS are these needed??
  mppi_pinocchio::Pose get_pose_handle(const mppi::observation_t& x);
  mppi_pinocchio::Pose get_pose_end_effector(const mppi::observation_t& x);
  //  geometry_msgs::PoseStamped get_pose_handle_ros(const mppi::observation_t& x);
  //  geometry_msgs::PoseStamped get_pose_end_effector_ros(
  //      const mppi::observation_t& x);
  //  geometry_msgs::PoseStamped get_pose_base(const mppi::observation_t& x);

  // configs
  mppi::config_t solver_config_;
  manipulation::Config manipulation_config_;


  // general params
  bool reference_set_;
  std::mutex reference_mutex_;
  std::unique_ptr<manipulation::PandaCost> local_cost_;
  mppi::reference_trajectory_t ref_;
  DynamicsParams dynamics_params_;
  mppi_pinocchio::RobotModel robot_model_;
  mppi_pinocchio::RobotModel object_model_;
  bool observation_set_ = false;

  // from ControllerRos, former base class
  mppi::cost_ptr cost_;
  mppi::dynamics_ptr dynamics_;
  std::shared_ptr<mppi::Solver> controller_ = nullptr;

  // ros
  double object_tolerance_;
  Eigen::VectorXd default_pose_;
  ReferenceScheduler reference_scheduler_;
  size_t last_ee_ref_id_;
  size_t last_ob_ref_id_;

  nav_msgs::Path optimal_path_;
  nav_msgs::Path optimal_base_path_;
  geometry_msgs::PoseStamped obstacle_pose_;
  geometry_msgs::PoseStamped ee_desired_pose_;
  visualization_msgs::Marker obstacle_marker_;

  // todo: gis these are currently unused (belong to the ControllerRos::get_input_state function)
  mppi::input_array_t u_opt_;
  mppi::observation_array_t x_opt_;
};

}  // namespace manipulation