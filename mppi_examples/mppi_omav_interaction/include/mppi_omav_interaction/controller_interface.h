/*!
 * @file     controller_interface.h
 * @author   Matthias Studiger
 * @date     10.04.2020
 * @version  1.0
 * @brief    description
 */

#ifndef OMAV_MPPI_CONTROLLER_INTERFACE_H
#define OMAV_MPPI_CONTROLLER_INTERFACE_H

#include <mppi_omav_interaction/cost.h>
#include <mppi_omav_interaction/cost_valve.h>
#include <mppi_omav_interaction/dynamics.h>
#include <mppi_omav_interaction/ros_conversions.h>
#include <mppi_ros/controller_interface.h>

#include <geometry_msgs/Pose.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int64.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <memory>

#include <eigen_conversions/eigen_msg.h>

#include <ros/ros.h>

namespace omav_interaction {

enum class InteractionTask { None, Shelf, Valve };

class OMAVControllerInterface : public mppi_ros::ControllerRos {
 public:
  explicit OMAVControllerInterface(ros::NodeHandle &nh,
                                   ros::NodeHandle &nh_public)
      : ControllerRos(nh, nh_public), task_(InteractionTask::None) {}

  ~OMAVControllerInterface() = default;

  void setTask(const std::string &str);
  InteractionTask getTask() const { return task_; }

  bool init_ros() override;

  void publish_ros() override;

  void publish_all_trajectories() override;

  void publish_optimal_rollout() override;

  bool update_reference() override;

  bool update_cost_param_shelf(const OMAVInteractionCostParam &cost_param);
  bool update_cost_param_valve(const OMAVInteractionCostValveParam &cost_param);

  bool set_initial_reference(const observation_t &x);

  void manually_shift_input(const int &index);

  void updateValveReference(const double &ref_angle);

  bool get_current_trajectory(
      trajectory_msgs::MultiDOFJointTrajectory *current_trajectory_msg) const;

  /**
   * @brief      Gets pointers to all optimizer dynamics (used in all threads as
   * well as optimal rollout sampling).
   *
   * @param      omav_dynamics_v  Vector of pointers to dynamics
   */
  void getDynamicsPtr(
      std::vector<std::shared_ptr<OMAVVelocityDynamics>> &omav_dynamics_v);

  void setDampingFactor(const double &d);

  void setHoldTime(const double &t) { get_controller()->setHoldTime(t); }

 private:
  bool set_controller(std::shared_ptr<mppi::PathIntegral> &controller) override;

  void desired_pose_callback(const geometry_msgs::PoseStampedConstPtr &msg);

  void mode_callback(const std_msgs::Int64ConstPtr &msg);

  void object_reference_callback(const geometry_msgs::PoseStampedConstPtr &msg);

  void publish_trajectory(const mppi::observation_array_t &x_opt,
                          const mppi::input_array_t &u_opt,
                          const mppi::observation_t &x0_opt,
                          const std::vector<double> &tt);

  void publishShelfInfo(const std_msgs::Header &header) const;
  void publishHookPos(const std_msgs::Header &header) const;

  void toMultiDofJointTrajectory(
      trajectory_msgs::MultiDOFJointTrajectory &t) const;

 public:
  mppi::SolverConfig config_;
  std::shared_ptr<OMAVInteractionCost> cost_shelf_;
  std::shared_ptr<OMAVInteractionCostValve> cost_valve_;

 private:
  InteractionTask task_;

  bool reference_set_ = false;
  bool detailed_publishing_;

  OMAVInteractionCostParam cost_param_shelf_;
  OMAVInteractionCostValveParam cost_param_valve_;

  ros::Publisher cmd_multi_dof_joint_trajectory_pub_;
  ros::Publisher optimal_rollout_des_publisher_;
  ros::Publisher cost_publisher_;
  ros::Publisher object_state_publisher_;
  ros::Publisher normalized_force_publisher_;
  ros::Publisher mppi_reference_publisher_;
  ros::Publisher optimal_linear_input_publisher_;
  ros::Publisher optimal_angular_input_publisher_;
  ros::Publisher optimal_rollout_lin_vel_;
  ros::Publisher optimal_rollout_ang_vel_;
  ros::Publisher pub_marker_hook_;

  ros::Subscriber reference_subscriber_;
  ros::Subscriber mode_subscriber_;
  ros::Subscriber object_reference_subscriber_;

  std::string robot_description_raisim_;
  std::string robot_description_pinocchio_;
  std::string object_description_;

  std::mutex reference_mutex_;
  mppi::reference_trajectory_t ref_;

  observation_array_t xx_opt_;
  input_array_t uu_opt_;
  std::vector<double> tt_opt_;
  observation_t x0_;
  Eigen::Vector3d com_hook_;

  sensor_msgs::JointState object_state_;

  trajectory_msgs::MultiDOFJointTrajectory current_trajectory_msg_;
  bool published_trajectory_ = false;

  double damping_ = 5.0;
};
}  // namespace omav_interaction

#endif