//
// Created by studigem on 13.04.21.
//

#ifndef MPPI_OMAV_VELOCITY_OMAV_VELOCITY_CONTROL_H_
#define MPPI_OMAV_VELOCITY_OMAV_VELOCITY_CONTROL_H_

#include "mppi_omav_velocity/controller_interface.h"
#include "mppi_omav_velocity/dynamics_ros.h"
#include "mppi_omav_velocity/ros_conversions.h"

#include <chrono>
#include <memory>
#include <ros/ros.h>
#include <vector>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include <mppi_omav_interaction/MPPIOmavGoalConfig.h>

namespace omav_velocity {
class OmavTrajectoryGenerator {
public:
  OmavTrajectoryGenerator(const ros::NodeHandle &nh,
                          const ros::NodeHandle &private_nh);
  ~OmavTrajectoryGenerator();

  void get_odometry(observation_t &x);
  bool odometry_bool_;
  bool reset_object_;
  void get_start_position(observation_t &x);

private:
  void initializeSubscribers();
  void initializePublishers();
  void odometryCallback(const nav_msgs::OdometryConstPtr &odometry_msg);
  void goalTranslationCallback(const geometry_msgs::Vector3 &translation);
  void goalRotationCallback(const geometry_msgs::Vector3 &rotation);
  bool takeOffSrv(std_srvs::Empty::Request &request,
                  std_srvs::Empty::Response &response);
  bool executeTrajectorySrv(std_srvs::Empty::Request &request,
                            std_srvs::Empty::Response &response);
  bool homingSrv(std_srvs::Empty::Request &request,
                 std_srvs::Empty::Response &response);
  bool landingSrv(std_srvs::Empty::Request &request,
                  std_srvs::Empty::Response &response);

  void GoalParamCallback(mppi_omav_interaction::MPPIOmavGoalConfig &config,
                         uint32_t level);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // subscribers
  ros::Subscriber odometry_sub_;
  ros::Subscriber goal_translation_sub_;
  ros::Subscriber goal_rotation_sub_;

  // publishers
  ros::Publisher reference_publisher_;
  ros::Publisher indicator_publisher_;

  // Services:
  ros::ServiceServer take_off_srv_;
  ros::ServiceServer execute_trajectory_srv_;
  ros::ServiceServer homing_srv_;
  ros::ServiceServer landing_srv_;

  // Odometry Variable
  mav_msgs::EigenOdometry current_odometry_;
  // Dynamics Reconfigure
  dynamic_reconfigure::Server<mppi_omav_interaction::MPPIOmavGoalConfig>
      goal_param_server_;

  // Indicator Message
  std_msgs::Int64 indicator;

  Eigen::VectorXd goal_pose_;

  // Start position
  Eigen::VectorXd start_position_;
};
} // namespace omav_velocity

#endif // MPPI_OMAV_VELOCITY_OMAV_VELOCITY_CONTROL_H_
