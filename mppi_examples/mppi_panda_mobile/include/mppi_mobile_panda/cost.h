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

namespace mobile_panda{

 class PandaCost: public mppi::CostBase{
 public:
   PandaCost(){
     std::string urdf_path = ros::package::getPath("mppi_mobile_panda");
     urdf_path += "/resources/panda/panda.urdf";
     std::cout << "Parsing model from: " << urdf_path << std::endl;
     pinocchio::urdf::buildModel(urdf_path, model_);
     data_ = pinocchio::Data(model_);
     frame_id_ = model_.getFrameId(tracked_frame_);
     Q_linear_ = Eigen::Matrix3d::Identity();
     Q_angular_ = Eigen::Matrix3d::Identity();
   };
   ~PandaCost() = default;

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

   bool elbow_up_ = false;

   pinocchio::SE3 pose_on_receipt_;
   double receipt_time_;
   double execution_time_;


 public:
   cost_ptr create() override {
     return std::make_shared<PandaCost>();
   }

   cost_ptr clone() const override {
     return std::make_shared<PandaCost>(*this);
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


   void set_reference(const pinocchio::SE3& pose, const double receipt_time, const Eigen::VectorXd& x){
     pose_current_ = get_current_pose(x);
     pose_reference_ = pose;
     receipt_time_ = receipt_time;
     time_reference();
   }

   pinocchio::SE3 get_current_pose(const Eigen::VectorXd& x){
     pinocchio::forwardKinematics(model_, data_, x.head<7>());
     pinocchio::updateFramePlacements(model_, data_);
     Eigen::Matrix3d base_rotation(Eigen::AngleAxisd(x(9), Eigen::Vector3d::UnitZ())) ;
     Eigen::Vector3d base_translation(x(7), x(8), 0.0);
     pinocchio::SE3 base_tf = pinocchio::SE3(base_rotation, base_translation);
     return base_tf.act(data_.oMf[frame_id_]);
   }

   void set_obstacle(const pinocchio::SE3& pose){
     pose_obstacle_ = pose;
     obstacle_set_ = true;
   }

   void time_reference(){
     double linear_error = pose_current_.actInv(pose_reference_).translation().norm();
     constexpr double max_linear_speed = 0.1;
     execution_time_ = linear_error / max_linear_speed;
     pose_on_receipt_ = pose_current_;
     std::cout << "Timed reference when pose current is: " << pose_on_receipt_ << std::endl;
     std::cout << "Execution time is: " << execution_time_ << std::endl;
     //pinocchio::Motion err = pinocchio::log6(pose_current_.actInv(pose_reference_));
     //double linear_scaling = err.linear().minCoeff() > 0.5 ? 1.0 : err.linear().minCoeff() / 0.5;
     //double angular_scaling = err.angular().minCoeff() > 0.1 ? 1.0 : err.linear().minCoeff()/0.1;
     //double linear
   }

   cost_t get_stage_cost(const Eigen::VectorXd& x, const double time=0) override {
    static double linear_cost = 0;
    static double angular_cost = 0;
    static double obstacle_cost = 0;
    static double elbow_cost = 0;
    static double reach_cost;
    double min_obstacle_dist = std::numeric_limits<double>::max();

    double lambda = (time - receipt_time_) >= execution_time_ ? 1 : (time - receipt_time_)/execution_time_;
    auto pose_reference_intermediate = pinocchio::SE3::Interpolate(pose_on_receipt_, pose_reference_, lambda);
    pose_current_ = get_current_pose(x);
    pinocchio::Motion err = pinocchio::log6(pose_current_.actInv(pose_reference_intermediate));

    linear_cost = err.linear().transpose() * Q_linear_ * err.linear();
    angular_cost = err.angular().transpose() * Q_angular_ * err.angular();

    if (obstacle_set_){
      double obstacle_dist = 0;
//      for(auto const& frame_name : model_.names){
//        obstacle_dist = (data_.oMi[model_.getJointId(frame_name)].translation() - pose_obstacle_.translation()).norm();
//        if (obstacle_dist < min_obstacle_dist)
//          min_obstacle_dist = obstacle_dist;
//      }

      // check separately for the controlled frame
      obstacle_dist = (pose_current_.translation() - pose_obstacle_.translation()).norm();
      if (obstacle_dist < min_obstacle_dist)
        min_obstacle_dist = obstacle_dist;

      if (min_obstacle_dist < obstacle_radius_)
        obstacle_cost =  Q_obst_;
      else
        obstacle_cost = 0;
    }

    if (elbow_up_)
      elbow_cost = 0;

    if (data_.oMf[frame_id_].translation().head<2>().norm() > 1.0)
      reach_cost = Q_reach_;
    else
      reach_cost = 0.0;

    return linear_cost + angular_cost + obstacle_cost + elbow_cost + reach_cost;
  }


  // Just for debugging
  pinocchio::SE3 get_pose_end_effector(const Eigen::VectorXd& x){
    pinocchio::forwardKinematics(model_, data_, x.head<7>());
    pinocchio::updateFramePlacements(model_, data_);
    return data_.oMf[frame_id_];
  }

 };
}
