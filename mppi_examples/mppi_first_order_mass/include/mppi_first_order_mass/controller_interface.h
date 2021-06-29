/*!
 * @file     controller_interface.h
 * @author   Giuseppe Rizzi
 * @date     08.09.2020
 * @version  1.0
 * @brief    description
 */
#pragma once
#include <mppi_ros/controller_interface.h>
#include "mppi_first_order_mass/cost.h"
#include "mppi_first_order_mass/dynamics.h"

#include <geometry_msgs/Point.h>

namespace fom {

class FOMControllerInterface : public mppi_ros::ControllerRos {
 public:
  explicit FOMControllerInterface(ros::NodeHandle& nh);
  ~FOMControllerInterface() = default;

  bool init_ros() override { return true; };
  void publish_ros() override;
  bool update_reference() override;

 private:
  bool set_controller(mppi::solver_ptr& controller) override;
  void goal_cb(const geometry_msgs::PointConstPtr& msg);

 public:
  mppi::config_t config_;

 private:
  bool reference_set_ = false;
  mppi::reference_trajectory_t ref_;
  ros::Subscriber reference_subscriber_;
};

}  // namespace fom