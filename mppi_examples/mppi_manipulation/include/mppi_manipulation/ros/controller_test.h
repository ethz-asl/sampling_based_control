//
// Created by giuseppe on 09.02.21.
//

#pragma once

//
// Created by giuseppe on 22.01.21.
//

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "mppi_manipulation/controller_interface.h"

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>

#include <ros/ros.h>

namespace manipulation {

class ControllerTest
    : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface> {
 public:

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override {};
  void update(const ros::Time&, const ros::Duration& period) override {};
  void stopping(const ros::Time& time) override {};

 private:

 private:
  std::unique_ptr<manipulation::PandaControllerInterface> man_interface_;

};

}  // namespace manipulation