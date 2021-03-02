/*!
 * @file     pendulum_cart_cost.h
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#pragma once

#include <math.h>
#include <mppi/cost/cost_base.h>
#include <mppi_pinocchio/model.h>
#include <ros/ros.h>

#include <ros/package.h>

namespace manipulation {

struct PandaCostParam {
  double Qt;  // translation cost
  double Qt2;
  double Qr;  // rotation cost
  double Qr2;
  double Qo;   // obstacle cost
  double Qos;  // obstacle cost slope
  double Qc;   // contact cost
  double ro;   // obstacle radius
  double max_reach;
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

class PandaCost : public mppi::CostBase {
 public:
  PandaCost() : PandaCost("", "", PandaCostParam()){};
  PandaCost(const std::string& robot_description,
            const std::string& object_description, const PandaCostParam& param,
            bool fixed_base = true);
  ~PandaCost() = default;

 private:
  bool fixed_base_;
  std::string robot_description_;
  std::string object_description_;
  PandaCostParam param_;

  mppi_pinocchio::RobotModel robot_model_;
  mppi_pinocchio::RobotModel object_model_;

  std::string handle_frame_ = "handle_link";
  std::string tracked_frame_ = "panda_grasp";
  std::string arm_base_frame_ = "panda_link0";
  int frame_id_;
  int arm_base_frame_id_;
  Eigen::Matrix<double, 6, 1> error_;

 public:
  cost_ptr create() override {
    return std::make_shared<PandaCost>(robot_description_, object_description_,
                                       param_, fixed_base_);
  }
  cost_ptr clone() const override { return std::make_shared<PandaCost>(*this); }

  void set_linear_weight(const double k) { param_.Qt = k; }
  void set_angular_weight(const double k) { param_.Qr = k; }
  void set_obstacle_radius(const double r) { param_.ro = r; }

  cost_t compute_cost(const mppi::observation_t& x,
                      const mppi::reference_t& ref, const double t) override;
};
}  // namespace manipulation

std::ostream& operator<<(std::ostream& os,
                         const manipulation::PandaCostParam& param);