/*!
 * @file     cost.h
 * @author   Matthias Studiger
 * @date     10.04.2021
 * @version  1.0
 * @brief    description
 */

#pragma once

#include <mppi_omav_interaction/cost_common.h>

#include <mppi/cost/cost_base.h>
#include <ros/ros.h>
#include <cmath>
#include "mppi_pinocchio/model.h"

#include <ros/package.h>
#include <memory>
#include <string>

namespace omav_interaction {

struct OMAVInteractionCostShelfParam {
  double Q_x_omav;  // Distance to reference Cost
  double Q_y_omav;
  double Q_z_omav;

  double Q_orientation;  // Orientation Cost

  Eigen::Matrix<double, 6, 1> pose_costs, pose_costs_int;

  Eigen::Matrix<double, 6, 6>
      Q_pose, Q_pose_int;  // Pose cost, is constructed from Q_distance and Q_orientation

  double Q_object;  // Object Cost

  double Q_lin_vel;  // Velocity Costs
  double Q_ang_vel;

  Eigen::Matrix<double, 6, 1> vel_costs;
  Eigen::Matrix<double, 6, 6> Q_vel;

  double Q_leaving_field;  // leaving Field Costs

  double x_limit_min;  // Field Limits
  double x_limit_max;
  double y_limit_min;
  double y_limit_max;
  double z_limit_max;

  double floor_thresh;  // Save flying height
  double Q_floor;       // Near Floor Cost

  double Q_obstacle;  // Obstacle Costs
  double x_obstacle;
  double y_obstacle;

  double Q_handle_hook;       // Cost when handle hook distance is 1m
  double handle_hook_thresh;  // threshold so that handle hook cost is 0

  double Q_efficiency;
  double Q_force;

  double Q_torque;

  bool contact_bool;

  bool parse_from_ros(const ros::NodeHandle &nh);
  template <class A>
  bool getParameter(const ros::NodeHandle &nh, const std::string &id, A &val,
                    const bool &checkPositive);
};

class OMAVInteractionCostShelf : public mppi::CostBase {
 public:
  OMAVInteractionCostShelf(const std::string &robot_description_pinocchio,
                      const std::string &object_description,
                      OMAVInteractionCostShelfParam *param);

  ~OMAVInteractionCostShelf() = default;

 private:
  std::string robot_description_pinocchio_;
  std::string object_description_;
  OMAVInteractionCostShelfParam *param_ptr_;
  OMAVInteractionCostShelfParam param_;

  mppi_pinocchio::RobotModel robot_model_;
  mppi_pinocchio::RobotModel object_model_;
  // std::string hook_frame_ = "hook";
  // std::string handle_frame_ = "handle_ref";
  Frames frames_;

  Eigen::Matrix<double, 6, 1> delta_pose_;
  Eigen::Matrix<double, 6, 1> delta_pose_object;
  // double distance;
  // double distance_from_savezone;
  // double obstacle_cost;
  double mode_ = 0;
  double torque_angle_ = 0;
  Eigen::Vector3d hook_handle_vector_;
  Eigen::Vector3d tip_lin_velocity_;
  Eigen::Matrix<double, 6, 1> tip_velocity_;
  Eigen::Vector3d torque_;
  Eigen::Vector3d force_normed_;
  Eigen::Vector3d r_tip_omav_I_;
  Eigen::Vector3d handle_orthogonal_vector_;
  double distance_hook_handle_;

 public:
  Eigen::Matrix<double, kN_costs, 1> cost_vector_;
  double cost_;
  Eigen::Vector3d hook_pos_;

  cost_t compute_cost(const mppi::observation_t &x,
                      const mppi::reference_t &ref, const double t) override;

 private:
  cost_ptr create() override {
    return std::make_shared<OMAVInteractionCostShelf>(
        robot_description_pinocchio_, object_description_, param_ptr_);
  }

  cost_ptr clone() const override {
    return std::make_shared<OMAVInteractionCostShelf>(*this);
  }

  void compute_field_cost(const mppi::observation_t &x);
  // double distance_from_obstacle_cost(const mppi::observation_t &x);

  void compute_floor_cost(const double &omav_z);

  void compute_pose_cost(const Eigen::VectorXd &omav_state,
                         const Eigen::VectorXd &omav_reference);

  void compute_handle_hook_cost();

  void compute_object_cost(const Eigen::VectorXd &omav_state,
                           const Eigen::VectorXd &omav_reference);

  void compute_vectors();

  void compute_tip_velocity_cost(const Eigen::VectorXd &omav_state);

  void compute_torque_cost(const Eigen::VectorXd &omav_state);

  void compute_efficiency_cost(const Eigen::VectorXd &omav_state);

  void compute_velocity_cost(const Eigen::Vector3d &linear_velocity,
                             const Eigen::Vector3d &angular_velocity);
};

}  // namespace omav_interaction

std::ostream &operator<<(
    std::ostream &os, const omav_interaction::OMAVInteractionCostShelfParam &param);
