//
// Created by giuseppe on 01.03.21.
//

#pragma once
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/fwd.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

namespace mppi_pinocchio {

struct Pose {
  Eigen::Vector3d translation;
  Eigen::Quaterniond rotation;

  Pose() = default;
  Pose(Eigen::Vector3d t, Eigen::Quaterniond r) : translation(t), rotation(r){};
};

Pose operator*(const Pose&, const Pose&);
Eigen::Matrix<double, 6, 1> diff(const Pose&, const Pose&);

class RobotModel {
 public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  RobotModel() = default;
  ~RobotModel();

  RobotModel(const RobotModel& rhs);
  /**
   *
   * @param robot_description
   * @return
   */
  bool init_from_xml(const std::string& robot_description);

  /**
   *
   * @param q
   */
  void update_state(const Eigen::VectorXd& q);

  /**
   *
   * @param q
   * @param qd
   */
  void update_state(const Eigen::VectorXd& q, Eigen::VectorXd& qd);

  /**
   *
   * @param from_frame
   * @param to_frame
   * @param error
   */
  void get_error(const std::string& from_frame, const std::string& to_frame,
                 Vector6d& error) const;
  /**
   *
   * @param frame
   * @param rot
   * @param trans
   * @param error
   */
  void get_error(const std::string& frame, const Eigen::Quaterniond& rot,
                 const Eigen::Vector3d& trans, Vector6d& error) const;

  /**
   *
   * @param frame
   */
  Pose get_pose(const std::string& frame) const;

 private:
  pinocchio::Model* model_;
  pinocchio::Data* data_;
};
}  // namespace mppi_pinocchio
