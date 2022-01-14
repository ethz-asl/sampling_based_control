//
// Created by studigem on 13.04.21.
//

#ifndef MPPI_OMAV_TRAJECTORY_GENERATOR_H_
#define MPPI_OMAV_TRAJECTORY_GENERATOR_H_

#include "mppi_omav_interaction/controller_interface.h"
#include "mppi_omav_interaction/cost.h"
#include "mppi_omav_interaction/dynamics_ros.h"
#include "mppi_omav_interaction/ros_conversions.h"

#include <ros/ros.h>
#include <chrono>
#include <memory>
#include <vector>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include <mppi_omav_interaction/MPPIOmavCostConfig.h>
#include <mppi_omav_interaction/MPPIOmavCostValveConfig.h>
#include <mppi_omav_interaction/MPPIOmavReferenceConfig.h>

namespace omav_interaction {

class OmavTrajectoryGenerator {
 public:
  OmavTrajectoryGenerator(const ros::NodeHandle &nh,
                          const ros::NodeHandle &private_nh);
  ~OmavTrajectoryGenerator();

  bool get_state(observation_t &x);
  bool get_state(observation_t &x, double &timestamp, bool &is_new);

  bool set_target(const trajectory_msgs::MultiDOFJointTrajectoryPoint
                      &trajectory_msg_point);
  bool set_target(const mav_msgs::EigenTrajectoryPoint &target_state);
  void setCurrentTrajectory(
    const trajectory_msgs::MultiDOFJointTrajectory &current_trajectory);

  bool initialize_integrators(observation_t &x);

  bool getTargetStateFromTrajectory();

  bool rqt_cost_shelf_bool_ = false;
  bool rqt_cost_valve_bool_ = false;
  bool reset_object_ = false;
  bool first_trajectory_sent_ = false;
  bool shift_lock_ = false;
  double target_state_time_ = 0.0;
  ros::Time last_target_received_;

  OMAVInteractionCostParam rqt_cost_shelf_;
  OMAVInteractionCostValveParam rqt_cost_valve_;

  trajectory_msgs::MultiDOFJointTrajectory current_trajectory_;

 private:
  void initializeSubscribers();
  void initializePublishers();
  void odometryCallback(const nav_msgs::OdometryConstPtr &odometry_msg);
  void objectCallback(const sensor_msgs::JointState &object_msg);
  void TargetCallback(
      const trajectory_msgs::MultiDOFJointTrajectory &position_target_msg);
  void ReferenceParamCallback(
      mppi_omav_interaction::MPPIOmavReferenceConfig &config, uint32_t level);
  void CostParamCallback(mppi_omav_interaction::MPPIOmavCostConfig &config,
                         uint32_t level);
  void CostValveParamCallback(
      mppi_omav_interaction::MPPIOmavCostValveConfig &config, uint32_t level);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // subscribers
  ros::Subscriber odometry_sub_;
  ros::Subscriber object_state_sub_;
  ros::Subscriber position_target_sub_;
  ros::Subscriber attitude_target_sub_;

  // publishers
  ros::Publisher reference_publisher_;

  // Odometry Variable
  mav_msgs::EigenOdometry current_odometry_;

  // Object State Variable
  Eigen::Vector2d object_state_;
  // Dynamics Reconfigure
  dynamic_reconfigure::Server<mppi_omav_interaction::MPPIOmavReferenceConfig>
      reference_param_server_;
  dynamic_reconfigure::Server<mppi_omav_interaction::MPPIOmavCostConfig>
      cost_param_server_;
  dynamic_reconfigure::Server<mppi_omav_interaction::MPPIOmavCostValveConfig>
      cost_valve_param_server_;

  // Target Variables
  mav_msgs::EigenTrajectoryPoint target_state_;

  observation_t rqt_odometry;

  bool object_valid_ = false;
  bool odometry_valid_ = false;
  bool is_new_odom_ = false;
  bool trajectory_available_ = false;
};
}  // namespace omav_interaction

#endif  // MPPI_OMAV_TRAJECTORY_GENERATOR_H_
