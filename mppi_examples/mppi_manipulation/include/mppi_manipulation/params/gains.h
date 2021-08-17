//
// Created by giuseppe on 09.08.21.
//

#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include "mppi_manipulation/dimensions.h"

namespace manipulation {

template <int N>
struct gain_pair {
  gain_pair(double kp, double kd, double ki = 0, double i_max = 0.0) {
    Kp.setConstant(kp);
    Kd.setConstant(kd);
    Ki.setConstant(ki);
    Imax.setConstant(i_max);
  }
  Eigen::Matrix<double, N, 1> Kp;
  Eigen::Matrix<double, N, 1> Kd;
  Eigen::Matrix<double, N, 1> Ki;
  Eigen::Matrix<double, N, 1> Imax;

  bool parse_from_ros(const ros::NodeHandle& nh,
                      const std::string& param_name) {
    std::vector<double> kp;
    if (!nh.getParam(param_name + "/kp", kp) || kp.size() != N) {
      ROS_ERROR_STREAM("Failed to parse " << param_name << "/kp or invalid");
      return false;
    }

    std::vector<double> kd;
    if (!nh.getParam(param_name + "/kd", kd) || kd.size() != N) {
      ROS_ERROR_STREAM("Failed to parse " << param_name << "/kd or invalid");
      return false;
    }

    std::vector<double> ki;
    if (!nh.getParam(param_name + "/ki", ki) || ki.size() != N) {
      ROS_ERROR_STREAM("Failed to parse " << param_name << "/ki or invalid");
      return false;
    }

    std::vector<double> i_max;
    if (!nh.getParam(param_name + "/i_max", i_max) || i_max.size() != N) {
      ROS_ERROR_STREAM("Failed to parse " << param_name << "/i_max or invalid");
      return false;
    }

    for (size_t i = 0; i < N; i++) {
      Kp[i] = kp[i];
      Kd[i] = kd[i];
      Ki[i] = ki[i];
      Imax[i] = i_max[i];
    }
    return true;
  }
};

struct PIDGains {
  PIDGains()
      : base_gains(0, 1000), arm_gains(0.0, 10.0), gripper_gains(100.0, 50.0){};

  gain_pair<BASE_DIMENSION> base_gains;
  gain_pair<ARM_DIMENSION> arm_gains;
  gain_pair<GRIPPER_DIMENSION> gripper_gains;

  bool init_from_ros(const ros::NodeHandle& nh, const std::string& prefix="");
};

}  // namespace manipulation

std::ostream& operator<<(std::ostream& os, const manipulation::PIDGains& gains);
