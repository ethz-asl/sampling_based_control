//
// Created by giuseppe on 01.03.21.
//

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "mppi_pinocchio/model.h"

using namespace pinocchio;

namespace mppi_pinocchio {

Eigen::Matrix<double, 6, 1> diff(const Pose &p1, const Pose &p2) {
  return log6(SE3(p1.rotation, p1.translation)
                  .actInv(SE3(p2.rotation, p2.translation)))
      .toVector();
}

Eigen::Matrix<double, 6, 1> get_delta(const Pose &current_pose, const Pose &reference_pose) {
  pinocchio::SpecialEuclideanOperationTpl<3, double, 0> aSE3;
  pinocchio::SpecialEuclideanOperationTpl<3, double, 0>::ConfigVector_t pose_s,pose_g;
  pinocchio::SpecialEuclideanOperationTpl<3, double, 0>::TangentVector_t delta_u ;
  Eigen::Matrix<double, 6, 1> delta_u_vector;

  pose_s.head<3>(3) = current_pose.translation;
  pose_s.segment<3>(3) = current_pose.rotation.vec();
  pose_s(6) = current_pose.rotation.w();
  pose_g.head<3>(3) = reference_pose.translation;
  pose_g.segment<3>(3) = reference_pose.rotation.vec();
  pose_g(6) = reference_pose.rotation.w();

  aSE3.difference(pose_s, pose_g, delta_u);

  delta_u_vector(0) = delta_u(0);
  delta_u_vector(1) = delta_u(1);
  delta_u_vector(2) = delta_u(2);
  delta_u_vector(3) = delta_u(3);
  delta_u_vector(4) = delta_u(4);
  delta_u_vector(5) = delta_u(5);

    return delta_u_vector;
    }

Pose operator*(const Pose &p1, const Pose &p2) {
  Pose res;
  SE3 temp(
      SE3(p1.rotation, p1.translation).act(SE3(p2.rotation, p2.translation)));
  res.translation = temp.translation();
  res.rotation = temp.rotation();
  return res;
}

RobotModel::~RobotModel() {
  delete model_;
  delete data_;
};

// deep copy
RobotModel::RobotModel(const RobotModel &rhs) {
  model_ = new pinocchio::Model(*rhs.model_);
  data_ = new pinocchio::Data(*rhs.data_);
}

bool RobotModel::init_from_xml(const std::string &robot_description,
                               bool floating_base) {
  try {
    model_ = new Model();
    if (floating_base) {
      pinocchio::urdf::buildModelFromXML(
          robot_description, pinocchio::JointModelFreeFlyer(), *model_);
    } else {
      pinocchio::urdf::buildModelFromXML(robot_description, *model_);
    }

    data_ = new Data(*model_);
  } catch (std::runtime_error &exc) {
    std::cout << exc.what();
    return false;
  } catch (...) {
    std::cout << "Unknown exception caught while building model." << std::endl;
    return false;
  }
  return true;
}

void RobotModel::update_state(const Eigen::VectorXd &q) {
  forwardKinematics(*model_, *data_, q);
  updateFramePlacements(*model_, *data_);
}

void RobotModel::update_state(const Eigen::VectorXd &q, Eigen::VectorXd &qd) {
  forwardKinematics(*model_, *data_, q, qd);
  updateFramePlacements(*model_, *data_);
}
void RobotModel::get_error(const std::string &from_frame,
                           const std::string &to_frame, Vector6d &error) const {
  error = log6(data_->oMf[model_->getFrameId(to_frame)].actInv(
                   data_->oMf[model_->getFrameId(from_frame)]))
              .toVector();
}

void RobotModel::get_error(const std::string &frame,
                           const Eigen::Quaterniond &rot,
                           const Eigen::Vector3d &trans,
                           Vector6d &error) const {
  error = log6(data_->oMf[model_->getFrameId(frame)].actInv(SE3(rot, trans)))
              .toVector();
}

Pose RobotModel::get_pose(const std::string &frame) const {
  Pose pose;
  pose.translation = data_->oMf[model_->getFrameId(frame)].translation();
  pose.rotation = data_->oMf[model_->getFrameId(frame)].rotation();
  return pose;
}

} // namespace mppi_pinocchio