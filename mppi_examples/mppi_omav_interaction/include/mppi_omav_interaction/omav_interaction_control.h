//
// Created by studigem on 13.04.21.
//

#ifndef MPPI_OMAV_VELOCITY_OMAV_VELOCITY_CONTROL_H_
#define MPPI_OMAV_VELOCITY_OMAV_VELOCITY_CONTROL_H_

#include "mppi_omav_interaction/controller_interface.h"
#include "mppi_omav_interaction/cost.h"
#include "mppi_omav_interaction/dynamics_ros.h"
#include "mppi_omav_interaction/ros_conversions.h"

#include <chrono>
#include <memory>
#include <ros/ros.h>
#include <vector>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include <mppi_omav_interaction/MPPIOmavCostConfig.h>
#include <mppi_omav_interaction/MPPIOmavReferenceConfig.h>

namespace omav_interaction {

class OmavTrajectoryGenerator {
public:
  OmavTrajectoryGenerator(const ros::NodeHandle &nh,
                          const ros::NodeHandle &private_nh);

  ~OmavTrajectoryGenerator();

  void get_odometry(observation_t &x);

  void initialize_integrators(observation_t &x);

  bool odometry_bool_;

  bool rqt_cost_bool_ = false;

  bool reset_object_ = false;

  bool shift_input_ = true;

  OMAVInteractionCostParam rqt_cost_;

private:
  void initializeSubscribers();

  void initializePublishers();

  void odometryCallback(const nav_msgs::OdometryConstPtr &odometry_msg);

  void objectCallback(const sensor_msgs::JointState &object_msg);

  void TargetCallback(
      const trajectory_msgs::MultiDOFJointTrajectory &position_target_msg);

  void
  ReferenceParamCallback(mppi_omav_interaction::MPPIOmavReferenceConfig &config,
                         uint32_t level);

  void CostParamCallback(mppi_omav_interaction::MPPIOmavCostConfig &config,
                         uint32_t level);

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
  // Target Variables
  mav_msgs::EigenTrajectoryPoint target_state_;
  // Object State Variable
  Eigen::Vector2d object_state_;
  // Dynamics Reconfigure
  dynamic_reconfigure::Server<mppi_omav_interaction::MPPIOmavReferenceConfig>
      reference_param_server_;
  dynamic_reconfigure::Server<mppi_omav_interaction::MPPIOmavCostConfig>
      cost_param_server_;

  observation_t rqt_odometry;
};
} // namespace omav_interaction

#endif // MPPI_OMAV_VELOCITY_OMAV_VELOCITY_CONTROL_H_
