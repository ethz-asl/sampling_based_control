/*!
 * @file     controller_interface.h
 * @author   Giuseppe Rizzi
 * @date     08.09.2020
 * @version  1.0
 * @brief    description
 */
#pragma once
#include <mppi_ros/controller_interface.h>
#include "mppi_pole_cart/cost.h"
#include "mppi_pole_cart/dynamics.h"

#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>

namespace pole_cart {

class PoleCartControllerInterface : public mppi_ros::ControllerRos {
 public:
  explicit PoleCartControllerInterface(ros::NodeHandle& nh)
      : ControllerRos(nh){};
  ~PoleCartControllerInterface() = default;

  bool init_ros() override { return true; };
  void publish_ros() override;
  bool update_reference() override;

 private:
  bool set_controller(std::shared_ptr<mppi::PathIntegral>& controller) override;

 public:
  mppi::SolverConfig config_;

 private:
  bool reference_set_ = false;
  mppi::reference_trajectory_t ref_;
};

}  // namespace pole_cart