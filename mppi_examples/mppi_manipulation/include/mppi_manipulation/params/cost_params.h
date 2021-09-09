//
// Created by giuseppe on 09.08.21.
//

#pragma once
#include <mppi_pinocchio/model.h>
#include <ros/ros.h>

namespace manipulation {

struct CostParams {
  std::string robot_description;
  std::string object_description;

  double Qreg;  // robot velocity regularization
  double Qt;    // translation cost
  double Qt2;
  double Qr;  // rotation cost
  double Qr2;
  double Qo;   // obstacle cost
  double Qos;  // obstacle cost slope
  double Qc;   // contact cost
  double ro;   // obstacle radius
  double max_reach;
  double min_dist;  // min distance from base for collision avoidance
  double Q_reach;
  double Q_reachs;
  double Q_obj;
  double Q_tol;
  mppi_pinocchio::Pose grasp_offset;
  double Q_joint_limit;
  double Q_joint_limit_slope;
  std::vector<double> upper_joint_limits;
  std::vector<double> lower_joint_limits;

  // frames
  std::string handle_frame = "handle_link";
  std::string tracked_frame = "panda_grasp";
  std::string arm_base_frame = "panda_link0";

  bool init_from_ros(const ros::NodeHandle& nh);
};

}  // namespace manipulation

std::ostream& operator<<(std::ostream& os,
                         const manipulation::CostParams& param);
