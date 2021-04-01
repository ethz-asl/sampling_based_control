/*!
 * @file    dynamics_ros.h
 * @author  Matthias Studiger
 * @date    19.03.2021
 * @version 1.0
 * @brief   description
 */

#pragma once

#include "mppi_omav_raisim/dynamics.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <string>

namespace omav_raisim {

class OMAVRaisimDynamicsRos : public OMAVRaisimDynamics {
 public:
  OMAVRaisimDynamicsRos(const ros::NodeHandle &nh,
                        const std::string &robot_description, const double dt);
  ~OMAVRaisimDynamicsRos() = default;

 public:
  void reset_to_default();
  void publish_ros();

 private:
  ros::NodeHandle nh_;
  // State Publisher will be unnecessary once we use it in the omav structure,
  // only used for the rviz visualization.
  ros::Publisher vis_publisher_;
  ros::Publisher goal_publisher_;
  visualization_msgs::Marker goal_marker_;
  visualization_msgs::Marker omav_marker_;
};
}  // namespace omav_raisim
