/*!
 * @file     controller_interface.h
 * @author   Matthias Studiger
 * @date     10.04.2020
 * @version  1.0
 * @brief    description
 */

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <mppi_ros/controller_interface.h>

#include <memory>
#include <std_msgs/Int64.h>

namespace omav_velocity {

class OMAVControllerInterface : public mppi_ros::ControllerRos {
public:
  explicit OMAVControllerInterface(ros::NodeHandle &nh) : ControllerRos(nh) {}
  ~OMAVControllerInterface() = default;

  bool init_ros() override;
  void publish_ros() override;
  void publish_trajectories() override;
  void publish_optimal_rollout() override;
  bool update_reference() override;

private:
  bool set_controller(std::shared_ptr<mppi::PathIntegral> &controller) override;
  void desired_pose_callback(const geometry_msgs::PoseStampedConstPtr &msg);

public:
  mppi::SolverConfig config_;

private:
  bool reference_set_ = false;

  ros::Subscriber reference_subscriber_;

  geometry_msgs::PoseStamped desired_pose_;

  std::mutex reference_mutex_;
  mppi::reference_trajectory_t ref_;
};
} // namespace omav_velocity
