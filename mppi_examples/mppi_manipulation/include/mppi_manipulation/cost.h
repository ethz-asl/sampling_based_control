/*!
 * @file     pendulum_cart_cost.h
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#pragma once

#include <math.h>
#include <mppi/core/cost.h>
#include <mppi_pinocchio/model.h>
#include <ros/ros.h>

#include <ros/package.h>
#include "mppi_manipulation/params/cost_params.h"

namespace manipulation {

struct PandaCostParam {
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

  bool parse_from_ros(const ros::NodeHandle& nh);
};

class PandaCost : public mppi::Cost {
 public:
  PandaCost() : PandaCost(CostParams()){};
  PandaCost(const CostParams& param);
  ~PandaCost() = default;

  // debug only
  inline const mppi_pinocchio::RobotModel& robot() const {
    return robot_model_;
  }
  inline const mppi_pinocchio::RobotModel& object() const {
    return object_model_;
  }

 private:
  CostParams params_;

  mppi_pinocchio::RobotModel robot_model_;
  mppi_pinocchio::RobotModel object_model_;

  std::string handle_frame_ = "handle_link";
  std::string tracked_frame_ = "panda_grasp";
  std::string arm_base_frame_ = "panda_link0";
  int frame_id_;
  int arm_base_frame_id_;
  Eigen::Matrix<double, 6, 1> error_;
  Eigen::Vector3d distance_vector_;

 public:
  mppi::cost_ptr create() override {
    return std::make_shared<PandaCost>(params_);
  }
  mppi::cost_ptr clone() const override {
    return std::make_shared<PandaCost>(*this);
  }

  void set_linear_weight(const double k) { params_.Qt = k; }
  void set_angular_weight(const double k) { params_.Qr = k; }
  void set_obstacle_radius(const double r) { params_.ro = r; }

  mppi::cost_t compute_cost(const mppi::observation_t& x,
                            const mppi::input_t& u,
                            const mppi::reference_t& ref,
                            const double t) override;
};
}  // namespace manipulation

std::ostream& operator<<(std::ostream& os,
                         const manipulation::PandaCostParam& param);