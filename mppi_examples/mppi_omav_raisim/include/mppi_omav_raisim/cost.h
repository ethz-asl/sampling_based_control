/*!
 * @file     cost.h
 * @author   Matthias Studiger
 * @date     22.03.2021
 * @version  1.0
 * @brief    description
 */

#pragma once

#include <math.h>
#include <mppi/cost/cost_base.h>
#include <ros/ros.h>

#include <memory>
#include <ros/package.h>
#include <string>

namespace omav_raisim {

struct OMAVRaisimCostParam {
  double Q_distance_x;  // Distance to reference Cost
  double Q_distance_y;
  double Q_distance_z;

  double Q_orientation;  // Orientation Cost

  Eigen::Matrix<double, 6, 1> pose_costs;

  Eigen::Matrix<double, 6, 6> Q_pose;  // Pose cost, is constructed from Q_distance and Q_orientation

  double Q_leafing_field;  // Leafing Field Costs
  double x_limit;
  double y_limit;
  double z_limit;

  double Q_velocity_max;  // Maximum velocity cost
  double max_velocity;

  double Q_thrust_max;  // Maximum thrust cost
  double max_thrust;

  double Q_omega;  // Maximum angular velocity cost

  bool parse_from_ros(const ros::NodeHandle &nh);
};

class OMAVRaisimCost : public mppi::CostBase {
 public:
  OMAVRaisimCost() : OMAVRaisimCost("", OMAVRaisimCostParam()) {}
  OMAVRaisimCost(const std::string &robot_description,
                 const OMAVRaisimCostParam &param);
  ~OMAVRaisimCost() = default;

 private:
  std::string robot_description_;
  OMAVRaisimCostParam param_;
  Eigen::Matrix<double, 6, 1> delta_pose;

 private:
  cost_ptr create() override {
    return std::make_shared<OMAVRaisimCost>(robot_description_, param_);
  }
  cost_ptr clone() const override {
    return std::make_shared<OMAVRaisimCost>(*this);
  }

  cost_t compute_cost(const mppi::observation_t &x,
                      const mppi::reference_t &ref, const double t) override;
};

}  // namespace omav_raisim

std::ostream &operator<<(std::ostream &os,
                         const omav_raisim::OMAVRaisimCostParam &param);
