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

RobotModel::~RobotModel(){
  delete model_;
  delete data_;
};

// deep copy
RobotModel::RobotModel(const RobotModel& rhs) {
  model_ = new pinocchio::Model(*rhs.model_);
  data_ = new pinocchio::Data(*rhs.data_);
}

bool RobotModel::init_from_xml(const std::string& robot_description) {
  try {
    model_ = new Model();
    pinocchio::urdf::buildModelFromXML(robot_description, *model_);
    data_ = new Data(*model_);
  } catch (std::runtime_error& exc) {
    std::cout << exc.what();
    return false;
  } catch (...) {
    std::cout << "Unknown exception caught while building model." << std::endl;
    return false;
  }
  return true;
}

void RobotModel::update_state(const Eigen::VectorXd& q) {
  forwardKinematics(*model_, *data_, q);
  updateFramePlacements(*model_, *data_);
}

void RobotModel::update_state(const Eigen::VectorXd& q, Eigen::VectorXd& qd) {
  forwardKinematics(*model_, *data_, q, qd);
  updateFramePlacements(*model_, *data_);
}
void RobotModel::get_error(const std::string& from_frame, const std::string& to_frame,
                           Vector6d& error) const {
  error =
      log6(data_->oMf[model_->getFrameId(to_frame)].actInv(data_->oMf[model_->getFrameId(from_frame)]))
          .toVector();
}

void RobotModel::get_error(const std::string& frame, const Eigen::Quaterniond& rot,
                           const Eigen::Vector3d& trans, Vector6d& error) const {
  error = log6(data_->oMf[model_->getFrameId(frame)].actInv(SE3(rot, trans))).toVector();
}

Pose RobotModel::get_pose(const std::string& frame) const {
  Pose pose;
  pose.translation = data_->oMf[model_->getFrameId(frame)].translation();
  pose.rotation = data_->oMf[model_->getFrameId(frame)].rotation();
  return pose;
}

}  // namespace mppi_pinocchio