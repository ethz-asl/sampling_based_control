//
// Created by giuseppe on 09.08.21.
//
#pragma once
#include <ros/ros.h>
#include <Eigen/Core>

namespace manipulation {
struct FilterParams {
  // joint limits
  bool joint_limits = true;
  bool joint_limits_soft = false;
  double joint_limits_slack_multiplier = 0.0;
  Eigen::Matrix<double, 10, 1> q_min;
  Eigen::Matrix<double, 10, 1> q_max;

  // cartesian limits
  std::string urdf;
  bool cartesian_limits = false;
  bool cartesian_limits_soft = false;
  double cartesian_limits_slack_multiplier = 0.0;
  double max_reach = 0.8;
  double min_dist = 0.15;

  // articulated object avoidance
  bool object_avoidance = false;
  bool object_avoidance_soft = false;
  double object_avoidance_slack_multiplier = 0.0;
  double min_object_distance = 0.2;
  std::string object_urdf;
  std::string object_frame_id;

  // obstacle avoidance
  bool obstacle_avoidance = false;
  bool obstacle_avoidance_soft = false;
  double obstacle_avoidance_slack_multiplier = 0.0;
  std::string obstacle_frame_id;
  double min_obstacle_distance = 0.2;

  // input limits
  bool input_limits = true;
  Eigen::Matrix<double, 10, 1> u_min;
  Eigen::Matrix<double, 10, 1> u_max;

  // first derivative limits
  bool first_derivative_limits = false;
  Eigen::Matrix<double, 10, 1> ud_min;
  Eigen::Matrix<double, 10, 1> ud_max;

  // second derivative limits
  bool second_derivative_limits = false;
  Eigen::Matrix<double, 10, 1> udd_min;
  Eigen::Matrix<double, 10, 1> udd_max;

  // passivity constraint
  bool passivity_constraint = false;
  bool passivity_constraint_soft = false;
  double passivity_constraint_slack_multiplier = 0.0;
  double initial_tank_energy = 0.0;
  double min_tank_energy = 0.0;

  bool verbose = false;

  bool init_from_ros(ros::NodeHandle& nh);
};

}  // namespace manipulation

std::ostream& operator<<(std::ostream& os,
                         const manipulation::FilterParams& settings);
