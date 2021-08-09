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
  gain_pair(double kp, double kd) {
    Kp.setConstant(kp);
    Kd.setConstant(kd);
  }
  Eigen::Matrix<double, N, 1> Kp;
  Eigen::Matrix<double, N, 1> Kd;

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

    for (size_t i = 0; i < N; i++) {
      Kp[i] = kp[i];
      Kd[i] = kd[i];
    }
    return true;
  }
};

struct PDGains {
  PDGains()
      : base_gains(0, 1000), arm_gains(0.0, 10.0), gripper_gains(100.0, 50.0){};

  gain_pair<BASE_DIMENSION> base_gains;
  gain_pair<ARM_DIMENSION> arm_gains;
  gain_pair<GRIPPER_DIMENSION> gripper_gains;

  bool init_from_ros(const ros::NodeHandle& nh);
};

}  // namespace manipulation

std::ostream& operator<<(std::ostream& os, const manipulation::PDGains& gains);
