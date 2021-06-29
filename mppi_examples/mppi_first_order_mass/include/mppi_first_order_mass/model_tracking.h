/*!
 * @file     controller_interface.h
 * @author   Giuseppe Rizzi
 * @date     08.09.2020
 * @version  1.0
 * @brief    description
 */
#pragma once
#include <mppi_tools/model_tracking_controller.h>
#include <ros/ros.h>
#include "mppi_first_order_mass/cost.h"
#include "mppi_first_order_mass/dynamics.h"

#include <geometry_msgs/Point.h>

namespace fom {

class FOMModelTracking : public mppi_tools::ModelTrackingController {
 public:
  explicit FOMModelTracking(ros::NodeHandle& nh);
  ~FOMModelTracking() = default;

  void publish_ros();

 private:
  bool setup();
  void goal_cb(const geometry_msgs::PointConstPtr& msg);

 public:
  mppi::config_t config_;

 private:
  ros::NodeHandle nh_;
  mppi::reference_trajectory_t ref_;
  ros::Subscriber reference_subscriber_;

  geometry_msgs::Point state_;
  ros::Publisher state_publisher_;
};

}  // namespace fom