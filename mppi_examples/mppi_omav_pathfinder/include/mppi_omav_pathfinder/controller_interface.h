/*!
 * @file     controller_interface.h
 * @author   Matthias Studiger
 * @date     14.03.2021
 * @version  1.0
 * @brief    description
 */
#pragma once
#include <mppi_ros/controller_interface.h>
#include "mppi_OMAV_Pathfinder/cost.h"
#include "mppi_OMAV_Pathfinder/dynamics.h"

#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>

namespace omav_pathfinder {

class OMAV_PathfinderControllerInterface : public mppi_ros::ControllerRos {
 public:
  explicit OMAV_PathfinderControllerInterface(ros::NodeHandle& nh)
      : ControllerRos(nh){};
  ~OMAV_PathfinderControllerInterface() = default;

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

}  // namespace omav_pathfinder