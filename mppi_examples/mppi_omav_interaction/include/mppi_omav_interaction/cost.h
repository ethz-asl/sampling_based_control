/*!
 * @file     cost.h
 * @author   Matthias Studiger
 * @date     10.04.2021
 * @version  1.0
 * @brief    description
 */

#pragma once

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

  double Q_leafing_field; // Leafing Field Costs
  double x_limit;
  double y_limit;
  double z_limit;

  double Q_obstacle; // Obstacle Costs
  double x_obstacle;
  double y_obstacle;

  bool parse_from_ros(const ros::NodeHandle &nh);
};

class OMAVInteractionCost : public mppi::CostBase {
public:
  OMAVInteractionCost() : OMAVInteractionCost("", OMAVInteractionCostParam()) {}

  OMAVInteractionCost(const std::string &robot_description,
                      const OMAVInteractionCostParam &param);

  ~OMAVInteractionCost() = default;

private:
  std::string robot_description_;
  OMAVInteractionCostParam param_;
  Eigen::Matrix<double, 6, 1> delta_pose;
  Eigen::Matrix<double, 6, 1> delta_pose_object;
  double distance;
  double distance_from_savezone;
  double obstacle_cost;
  int mode;

private:
  cost_ptr create() override {
    return std::make_shared<OMAVInteractionCost>(robot_description_, param_);
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
