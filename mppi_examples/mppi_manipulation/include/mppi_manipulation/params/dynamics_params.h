//
// Created by giuseppe on 09.08.21.
//

#pragma once
#include "mppi_manipulation/params/filter_params.h"
#include "mppi_manipulation/params/gains.h"

namespace manipulation {

struct DynamicsParams {
  double dt;
  std::string robot_description;
  std::string object_description;
  PDGains gains;
  Eigen::VectorXd initial_state;

  bool has_filter;
  bool apply_filter;
  FilterParams filter_params;

  bool init_from_ros(ros::NodeHandle& nh, bool is_sim = false);
};

}  // namespace manipulation

std::ostream& operator<<(std::ostream& os,
                         const manipulation::DynamicsParams& params);
