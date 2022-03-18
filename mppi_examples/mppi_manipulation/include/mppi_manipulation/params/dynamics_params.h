//
// Created by giuseppe on 09.08.21.
//

#pragma once
#include "mppi_manipulation/params/gains.h"
#include "mppi_manipulation/config_no_ros.h"

namespace manipulation {

struct DynamicsParams {
  double dt;
  std::string robot_description;
  std::string object_description;
  std::string raisim_object_res_path = "/";
  std::string raisim_robot_res_path = "/";
  bool ignore_object_self_collision = true;
  bool ignore_panda_self_collision = true;
  PIDGains gains;
  Eigen::VectorXd initial_state;
  bool debug_prints;

  std::string articulation_joint;
  std::string object_handle_link;
  std::string object_handle_joint;

  bool init_from_ros(ros::NodeHandle& nh, bool is_sim = false);
  bool init_from_config(const manipulation::Config& config);
};

}  // namespace manipulation

std::ostream& operator<<(std::ostream& os,
                         const manipulation::DynamicsParams& params);
