//
// Created by giuseppe on 09.08.21.
//

#pragma once
#include "mppi_sliding/params/gains.h"

namespace manipulation {

struct DynamicsParams {
  double dt;
  std::string robot_description;
  std::vector<std::string> mug_description;
  PIDGains gains;
  Eigen::VectorXd initial_state;

  //geometry
  double cylinder_height;
  double cylinder_radius;
  double cylinder_z;
  std::vector<double> target_pos;
  double cylinder_mass;
  std::vector<double> table_position;
  double friction;

  // controller config
  bool fixed_base,update_geometry;

  bool init_from_ros(ros::NodeHandle& nh, bool is_sim = false);
};

}  // namespace manipulation

std::ostream& operator<<(std::ostream& os,
                         const manipulation::DynamicsParams& params);
