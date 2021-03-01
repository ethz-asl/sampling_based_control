//
// Created by giuseppe on 01.03.21.
//

#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <memory>

namespace mppi_pinocchio {

struct Pose {
  Eigen::Vector3d translation;
  Eigen::Quaterniond rotation;
};

class RobotModel {
 public:
  using model_t = std::unique_ptr<pinocchio::Model>;
  using data_t = std::unique_ptr<pinocchio::Data>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  RobotModel() = default;
  ~RobotModel() = default;

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
  void get_error(const std::string& from_frame, const std::string& to_frame, Vector6d& error) const;
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
  model_t model_;
  data_t data_;
};
}  // namespace mppi_pinocchio
