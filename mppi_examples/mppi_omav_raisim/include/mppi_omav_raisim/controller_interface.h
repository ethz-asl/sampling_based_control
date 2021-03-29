/*!
 * @file     controller_interface.h
 * @author   Matthias Studiger
 * @date     18.03.2020
 * @version  1.0
 * @brief    description
 */

#pragma once

#include <mppi_ros/controller_interface.h>

#include <memory>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int64.h>

namespace omav_raisim {

class OMAVControllerInterface : public mppi_ros::ControllerRos {
 public:
  explicit OMAVControllerInterface(ros::NodeHandle &nh) : ControllerRos(nh) {}
  ~OMAVControllerInterface() = default;

  bool init_ros() override { return true; };
  void publish_ros() override;
  bool update_reference() override;

 private:
  bool set_controller(std::shared_ptr<mppi::PathIntegral> &controller) override;

 public:
  mppi::SolverConfig config_;

 private:
  mppi::input_array_t u_opt_;
  mppi::observation_array_t x_opt_;

  bool reference_set_ = false;

  mppi::reference_trajectory_t ref_;
};
}  // namespace omav_raisim
