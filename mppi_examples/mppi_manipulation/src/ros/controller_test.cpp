//
// Created by giuseppe on 22.01.21.
//

#include <mppi_manipulation/ros/controller_test.h>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace manipulation {

bool ControllerTest::init(hardware_interface::RobotHW* robot_hw,
                                   ros::NodeHandle& node_handle) {

  man_interface_ = std::make_unique<manipulation::PandaControllerInterface>(node_handle);
  if (!man_interface_->init()) {
    ROS_ERROR("Failed to initialized manipulation interface");
    return false;
  }
  ROS_INFO("Controller initialized!");
  return true;
}
}  // namespace manipulation

PLUGINLIB_EXPORT_CLASS(manipulation::ControllerTest, controller_interface::ControllerBase)