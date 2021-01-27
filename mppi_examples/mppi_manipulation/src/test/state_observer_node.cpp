//
// Created by giuseppe on 23.01.21.
//

#include <ros/ros.h>
#include "mppi_manipulation/ros/state_observer.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "state_observer_test");
  ros::NodeHandle nh("~");

  bool fixed_base;
  nh.param<bool>("fixed_base", fixed_base, false);

  franka::RobotState franka_state;
  std::fill(franka_state.q.begin(), franka_state.q.end(), 0.0);
  std::fill(franka_state.dq.begin(), franka_state.dq.end(), 0.0);

  manipulation::StateObserver observer(nh, fixed_base);
  ROS_INFO("Sleeping 2.0 sec before starting the state observer.");
  ros::Duration(2.0).sleep();

  Eigen::VectorXd x;
  double start_time = ros::Time::now().toSec();
  ros::Rate rate(10);
  while (ros::ok()) {
    // update franka state in code
    franka_state.dq[0] = 0.1;
    franka_state.q[0] += franka_state.dq[0] * 0.1;

    observer.set_arm_state(franka_state);
    if (!observer.get_state(x)) {
      ROS_ERROR("Failed to update the current state.");
      return 0;
    }

    observer.publish_ros(x);

    ROS_INFO_STREAM_THROTTLE(1.0, observer.state_as_string(x));
    rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
