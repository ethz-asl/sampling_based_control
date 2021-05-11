/*!
 * @file     cost.h
 * @author   Matthias Studiger
 * @date     10.04.2021
 * @version  1.0
 * @brief    description
 */

#pragma once

#include "mppi_pinocchio/model.h"
#include <cmath>
#include <mppi/cost/cost_base.h>
#include <ros/ros.h>

#include <memory>
#include <ros/package.h>
#include <string>

namespace omav_interaction {

struct OMAVInteractionCostParam {
  double Q_distance_x; // Distance to reference Cost
  double Q_distance_y;
  double Q_distance_z;

  double Q_orientation; // Orientation Cost

  Eigen::Matrix<double, 6, 1> pose_costs;

  Eigen::Matrix<double, 6, 6>
      Q_pose; // Pose cost, is constructed from Q_distance and Q_orientation

  double Q_object_x; // Distance to reference Cost
  double Q_object_y;
  double Q_object_z;

  double Q_object_orientation; // Orientation Cost

  Eigen::Matrix<double, 6, 1> object_costs;
  Eigen::Matrix<double, 6, 6>
      Q_object; // Object cost, is constructed from Q_distance and Q_orientation

  double Q_lin_vel; // Velocity Costs
  double Q_ang_vel;

  Eigen::Matrix<double, 6, 1> vel_costs;
  Eigen::Matrix<double, 6, 6> Q_vel;

  double Q_force; // Force cost

  Eigen::Matrix<double, 3, 1> force_costs;
  Eigen::Matrix<double, 3, 3> Q_force_mat;

  double Q_leafing_field; // Leafing Field Costs

  double x_limit_min; // Field Limits
  double x_limit_max;
  double y_limit_min;
  double y_limit_max;
  double z_limit_max;

  double floor_thresh; // Save flying height
  double Q_floor;      // Near Floor Cost

  double Q_obstacle; // Obstacle Costs
  double x_obstacle;
  double y_obstacle;

  double Q_handle_hook;      // Cost when handle hook distance is 1m
  double handle_hook_thresh; // threshold so that handle hook cost is 0

  bool parse_from_ros(const ros::NodeHandle &nh);
};

class OMAVInteractionCost : public mppi::CostBase {
public:
  OMAVInteractionCost(const std::string &robot_description,
                      const std::string &robot_description_pinocchio,
                      const std::string &object_description,
                      const OMAVInteractionCostParam &param);

  ~OMAVInteractionCost() = default;

private:
  std::string robot_description_;
  std::string robot_description_pinocchio_;
  std::string object_description_;
  OMAVInteractionCostParam param_;

  mppi_pinocchio::RobotModel robot_model_;
  mppi_pinocchio::RobotModel object_model_;
  std::string hook_frame_ = "hook";
  std::string handle_frame_ = "handle";

  Eigen::Matrix<double, 6, 1> delta_pose;
  Eigen::Matrix<double, 6, 1> delta_pose_object;
  double distance;
  double distance_from_savezone;
  double obstacle_cost;
  double mode;
  Eigen::Vector3d hook_handle_vector;
  double distance_hook_handle;

private:
  cost_ptr create() override {
    return std::make_shared<OMAVInteractionCost>(robot_description_,
                                                 robot_description_pinocchio_,
                                                 object_description_, param_);
  }

  cost_ptr clone() const override {
    return std::make_shared<OMAVInteractionCost>(*this);
  }

  double distance_from_obstacle_cost(const mppi::observation_t &x);

  cost_t compute_cost(const mppi::observation_t &x,
                      const mppi::reference_t &ref, const double t) override;
};

} // namespace omav_velocity

std::ostream &
operator<<(std::ostream &os,
           const omav_interaction::OMAVInteractionCostParam &param);
