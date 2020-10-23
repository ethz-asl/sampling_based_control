/*!
 * @file     pendulum_cart_cost.h
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#pragma once
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <math.h>
#include <mppi/cost/cost_base.h>
#include <ros/ros.h>

#include <ros/package.h>

namespace panda{

class PandaCost: public mppi::CostBase{
 public:
   PandaCost(): PandaCost("", 1.0, 1.0, 0.0){};
   PandaCost(const std::string& robot_description, double linear_weight, double angular_weight, double obstacle_radius);
   ~PandaCost() = default;

 private:
   std::string robot_description_;
   double linear_weight_;
   double angular_weight_;
   double obstacle_radius_;

   pinocchio::Model model_;
   pinocchio::Data data_;

   std::string tracked_frame_ = "panda_grasp";
   int frame_id_;
   pinocchio::SE3 pose_current_;
   pinocchio::SE3 pose_reference_;
   Eigen::Matrix<double, 3, 3> Q_linear_;
   Eigen::Matrix<double, 3, 3> Q_angular_;

   double Q_obst_ = 100000;
   pinocchio::SE3 pose_obstacle_;

   Eigen::Matrix<double, 7, 1> joint_limits_lower_;
   Eigen::Matrix<double, 7, 1> joint_limits_upper_;

 public:
   cost_ptr create() override { return std::make_shared<PandaCost>(robot_description_, linear_weight_, angular_weight_, obstacle_radius_); }
   cost_ptr clone() const override { return std::make_shared<PandaCost>(*this); }

   void set_linear_weight(const double k){ Q_linear_ *= k; }
   void set_angular_weight(const double k){ Q_angular_ *= k; }
   void set_obstacle_radius(const double r){ obstacle_radius_ = r; }

   cost_t compute_cost(const mppi::observation_t& x, const mppi::reference_t& ref, const double t) override;

   pinocchio::SE3 get_pose_end_effector(const Eigen::VectorXd& x);

 };
}
