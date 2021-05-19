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
#include <mppi_omav_interaction/CostParam.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include <mppi_omav_interaction/MPPIOmavCostConfig.h>
#include <mppi_omav_interaction/MPPIOmavGoalConfig.h>

namespace omav_interaction {

class OmavTrajectoryGenerator {
public:
  OmavTrajectoryGenerator(const ros::NodeHandle &nh,
                          const ros::NodeHandle &private_nh);

  ~OmavTrajectoryGenerator();

  void get_odometry(observation_t &x);

  bool odometry_bool_;

  bool rqt_cost_bool_;

  bool reset_object_;

  OMAVInteractionCostParam rqt_cost_;

private:
  void initializeSubscribers();

  void initializePublishers();

  void odometryCallback(const nav_msgs::OdometryConstPtr &odometry_msg);

  bool takeOffSrv(std_srvs::Empty::Request &request,
                  std_srvs::Empty::Response &response);

  bool executeTrajectorySrv(std_srvs::Empty::Request &request,
                            std_srvs::Empty::Response &response);

  bool homingSrv(std_srvs::Empty::Request &request,
                 std_srvs::Empty::Response &response);

  void GoalParamCallback(mppi_omav_interaction::MPPIOmavGoalConfig &config,
                         uint32_t level);

  void CostParamCallback(mppi_omav_interaction::MPPIOmavCostConfig &config,
                         uint32_t level);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // subscribers
  ros::Subscriber odometry_sub_;

  // publishers
  ros::Publisher reference_publisher_;

  // Services:
  ros::ServiceServer take_off_srv_;
  ros::ServiceServer execute_trajectory_srv_;
  ros::ServiceServer homing_srv_;

  // Odometry Variable
  mav_msgs::EigenOdometry current_odometry_;
  // Dynamics Reconfigure
  dynamic_reconfigure::Server<mppi_omav_interaction::MPPIOmavGoalConfig>
      goal_param_server_;
  dynamic_reconfigure::Server<mppi_omav_interaction::MPPIOmavCostConfig>
      cost_param_server_;
};
} // namespace omav_velocity

#endif // MPPI_OMAV_VELOCITY_OMAV_VELOCITY_CONTROL_H_
