/*!
 * @file     pendulum_cart_cost.h
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#pragma once
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <math.h>
#include <mppi/cost/cost_base.h>
#include <ros/ros.h>

#include <ros/package.h>

namespace manipulation {

struct PandaCostParam {
  double Qt;  // translation cost
  double Qr;  // rotation cost
  double Qo;  // obstacle cost
  double Qc;  // contact cost
  double ro;  // obstacle radius
  pinocchio::SE3 grasp_offset;
};

class PandaCost : public mppi::CostBase {
 public:
  PandaCost() : PandaCost("", "", PandaCostParam()){};
  PandaCost(const std::string& robot_description, const std::string& object_description,
            const PandaCostParam& param, bool fixed_base=true);
  ~PandaCost() = default;

 private:
  bool fixed_base_;
  std::string robot_description_;
  std::string object_description_;
  PandaCostParam param_;

  pinocchio::Model model_;
  pinocchio::Data data_;

  // object
  pinocchio::Model object_model_;
  pinocchio::Data object_data_;
  int handle_idx_;

  std::string handle_frame_ = "handle_link";
  std::string tracked_frame_ = "panda_grasp";
  int frame_id_;
  pinocchio::Motion err_;
  pinocchio::SE3 pose_current_;
  pinocchio::SE3 pose_handle_;
  pinocchio::SE3 pose_reference_;
  pinocchio::SE3 pose_obstacle_;

  Eigen::Matrix<double, 7, 1> joint_limits_lower_;
  Eigen::Matrix<double, 7, 1> joint_limits_upper_;

 public:
  cost_ptr create() override {
    return std::make_shared<PandaCost>(robot_description_, object_description_, param_, fixed_base_);
  }
  cost_ptr clone() const override { return std::make_shared<PandaCost>(*this); }

  void set_linear_weight(const double k) { param_.Qt = k; }
  void set_angular_weight(const double k) { param_.Qr = k; }
  void set_obstacle_radius(const double r) { param_.ro = r; }

  cost_t compute_cost(const mppi::observation_t& x, const mppi::reference_t& ref,
                      const double t) override;

  pinocchio::SE3 get_pose_end_effector(const Eigen::VectorXd& x);
  pinocchio::SE3 get_pose_handle(const Eigen::VectorXd& x);
};
}  // namespace manipulation
