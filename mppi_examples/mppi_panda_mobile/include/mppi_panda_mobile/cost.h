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

namespace panda_mobile{

 class PandaMobileCost: public mppi::CostBase{
 public:
   PandaMobileCost(){
     // init model for forward kinematic
     std::string urdf_path = ros::package::getPath("mppi_panda_mobile");
     urdf_path += "/resources/panda/panda.urdf";
     std::cout << "Parsing model from: " << urdf_path << std::endl;
     pinocchio::urdf::buildModel(urdf_path, model_);
     data_ = pinocchio::Data(model_);
     frame_id_ = model_.getFrameId(tracked_frame_);
   }

   PandaMobileCost(const double linear_weight, const double angular_weight, const double obstacle_radius): PandaMobileCost(){
     Q_linear_ = Eigen::Matrix3d::Identity() * linear_weight;
     Q_angular_ = Eigen::Matrix3d::Identity() * angular_weight;
     obstacle_radius_ = obstacle_radius;
   };

   ~PandaMobileCost() = default;

 private:
   pinocchio::Model model_;
   pinocchio::Data data_;

   std::string tracked_frame_ = "panda_hand";
   int frame_id_;
   pinocchio::SE3 pose_current_;
   pinocchio::SE3 pose_reference_;
   Eigen::Matrix<double, 3, 3> Q_linear_;
   Eigen::Matrix<double, 3, 3> Q_angular_;

   double Q_obst_ = 100000;
   double Q_reach_ = 100000;
   pinocchio::SE3 pose_obstacle_;
   double obstacle_radius_;
   bool obstacle_set_ = false;


 public:
   cost_ptr create() override {
     return std::make_shared<PandaMobileCost>();
   }

   cost_ptr clone() const override {
     return std::make_shared<PandaMobileCost>(*this);
   }

   void set_linear_weight(const double k){
     Q_linear_ *= k;
   }

   void set_angular_weight(const double k){
     Q_angular_ *= k;
   }

   void set_obstacle_radius(const double r){
     obstacle_radius_ = r;
   }

   pinocchio::SE3 get_current_pose(const Eigen::VectorXd& x){
     pinocchio::forwardKinematics(model_, data_, x.head<7>());
     pinocchio::updateFramePlacements(model_, data_);
     Eigen::Matrix3d base_rotation(Eigen::AngleAxisd(x(9), Eigen::Vector3d::UnitZ())) ;
     Eigen::Vector3d base_translation(x(7), x(8), 0.0);
     pinocchio::SE3 base_tf = pinocchio::SE3(base_rotation, base_translation);
     return base_tf.act(data_.oMf[frame_id_]);
   }

   cost_t compute_cost(const mppi::observation_t& x, const mppi::reference_t& ref, const double t) override{
     static double linear_cost = 0;
     static double angular_cost = 0;
     static double obstacle_cost = 0;
     static double reach_cost;

     pose_current_ = get_current_pose(x);
     Eigen::Vector3d ref_t = ref.head<3>();
     Eigen::Quaterniond ref_q(ref.segment<4>(3));
     pose_reference_ = pinocchio::SE3(ref_q, ref_t);
     pinocchio::Motion err = pinocchio::log6(pose_current_.actInv(pose_reference_));

     linear_cost = err.linear().transpose() * Q_linear_ * err.linear();
     angular_cost = err.angular().transpose() * Q_angular_ * err.angular();

     double obstacle_dist = (pose_current_.translation() - ref.tail<3>()).norm();
     if (obstacle_dist < obstacle_radius_)
       obstacle_cost =  Q_obst_;
     else
       obstacle_cost = 0;

     if (data_.oMf[frame_id_].translation().head<2>().norm() > 1.0)
       reach_cost = Q_reach_;
     else
       reach_cost = 0.0;

     return linear_cost + angular_cost + obstacle_cost + reach_cost;
   }
 };
}
