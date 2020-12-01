/*!
 * @file     controller_ros.h
 * @author   Giuseppe Rizzi
 * @date     10.09.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#include "mppi_panda/controller_interface.h"
#include <robot_control/modeling/robot_wrapper.h>

#include <robot_control/modeling/robot_wrapper.h>
#include <robot_control/controllers/end_effector_controllers/task_space_controller.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/PoseStamped.h>
#include <realtime_tools/realtime_publisher.h>

#include <franka_hw/franka_state_interface.h>

#include <interactive_markers/interactive_marker_server.h>

namespace panda {

template<class StateInterface, class StateHandle>
class PandaControllerRosBase : public controller_interface::MultiInterfaceController<
    hardware_interface::EffortJointInterface,
    hardware_interface::VelocityJointInterface,
    StateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle, ros::NodeHandle& ctrl_handle) override;
  void starting(const ros::Time&) override;

  virtual bool addStateHandles(hardware_interface::RobotHW*) {};
  void publish_ros();

  Eigen::VectorXd getJointVelocities() const;
  Eigen::VectorXd getJointPositions() const;

 protected:
  mppi::observation_t x_;
  mppi::input_t u_;
  std::unique_ptr<panda::PandaControllerInterface> controller_;
  bool started_;

  std::vector<hardware_interface::JointStateHandle> state_handles_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  std::string joint_names_[7] = {"panda_joint1",
                                 "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5",
                                 "panda_joint6", "panda_joint7"};

  mppi::observation_t x_des_;
  Eigen::Matrix<double, 7, 1> tau_;
  Eigen::Matrix<double, 7, 7> Kp_;
  Eigen::Matrix<double, 7, 7> Kd_;
  std::shared_ptr<rc::RobotWrapper> robot_;

  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>> ee_pose_publisher_;
};

class PandaControllerRos : public PandaControllerRosBase<franka_hw::FrankaStateInterface, franka_hw::FrankaStateHandle> {
  bool addStateHandles(hardware_interface::RobotHW*);
  void update(const ros::Time&, const ros::Duration& period) override;
};

class PandaControllerRosSim : public PandaControllerRosBase<hardware_interface::JointStateInterface, hardware_interface::JointStateHandle> {
  bool addStateHandles(hardware_interface::RobotHW*);
  void update(const ros::Time&, const ros::Duration& period) override;
};
}

