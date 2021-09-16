//
// Created by giuseppe on 09.08.21.
//
#pragma once
#include <ros/ros.h>
#include <Eigen/Core>

namespace manipulation {
struct FilterParams {
  std::string urdf;

  // soften the hard constraints with slack variables
  bool joint_limits_soft = false;
  bool cartesian_limits_soft = false;
  bool passivity_constraint_soft = false;
  bool object_avoidance_soft = false;

  double joint_limits_slack_multiplier = 0.0;
  double cartesian_limits_slack_multiplier = 0.0;
  double passivity_constraint_slack_multiplier = 0.0;
  double object_avoidance_slack_multiplier = 0.0;

  Eigen::Matrix<double, 10, 1> u_min;
  Eigen::Matrix<double, 10, 1> u_max;
  Eigen::Matrix<double, 10, 1> ud_min;
  Eigen::Matrix<double, 10, 1> ud_max;
  Eigen::Matrix<double, 10, 1> udd_min;
  Eigen::Matrix<double, 10, 1> udd_max;

  Eigen::Matrix<double, 10, 1> q_min;
  Eigen::Matrix<double, 10, 1> q_max;

  double max_reach = 0.8;
  double min_dist = 0.15;

  // base object distance
  double min_object_distance = 0.2;
  std::string object_urdf;
  std::string object_frame_id;

  bool joint_limits = true;
  bool input_limits = true;
  bool cartesian_limits = false;
  bool object_avoidance = false;

  bool first_derivative_limits = false;
  bool second_derivative_limits = false;

  bool passivity_constraint = false;
  double initial_tank_energy = 0.0;
  double min_tank_energy = 0.0;

  bool verbose = false;

  bool init_from_ros(ros::NodeHandle& nh);
};

}  // namespace manipulation

std::ostream& operator<<(std::ostream& os,
                         const manipulation::FilterParams& settings);
