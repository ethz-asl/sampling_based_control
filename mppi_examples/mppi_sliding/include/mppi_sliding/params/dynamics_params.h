//
// Created by giuseppe on 09.08.21.
//

#pragma once
#include "mppi_sliding/params/gains.h"

namespace manipulation {

struct DynamicsParams {
  double dt;
  std::string robot_description;
  std::string object_description;
  std::string cylinder_description;
  PIDGains gains;
  Eigen::VectorXd initial_state;

  std::string articulation_joint;
  std::string object_handle_link;
  std::string object_handle_joint;

  //geometry
  double cylinder_height;
  double cylinder_radius;
  double cylinder_z;
  std::vector<double> table_position;

  bool init_from_ros(ros::NodeHandle& nh, bool is_sim = false);
};

}  // namespace manipulation

std::ostream& operator<<(std::ostream& os,
                         const manipulation::DynamicsParams& params);
