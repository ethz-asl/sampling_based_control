//
// Created by giuseppe on 09.08.21.
//
#pragma once
#include <ros/ros.h>
#include <Eigen/Core>

namespace manipulation {
struct FilterParams {
  std::string urdf;

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

  bool joint_limits = true;
  bool input_limits = true;
  bool cartesian_limits = false;

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
