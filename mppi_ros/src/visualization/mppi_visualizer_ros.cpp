/*!
 * @file     mppi_visualizer_ros.cpp
 * @author   Giuseppe Rizzi
 * @date     23.07.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_ros/visualization/mppi_visualizer_ros.h"

namespace mppi_ros{

void PIVisualizerRos::publish_ros(){
  for(size_t k=0; k<config_.rollouts; k++){
    rollouts_info_ros.cost_to_go[k] = rollouts_cost_(k);
    rollouts_info_ros.weights[k] = omega(k);
  }
  rollouts_info_publisher.publish(rollouts_info_ros);
}


void PIVisualizerRos::set_default_view(const std_msgs::BoolConstPtr& msg){
  std::cout << "Setting to default view mode? " << ((msg->data) ? "yes" : "no") << std::endl;
  set_default_view_if_true(msg->data);
}

void PIVisualizerRos::init_from_ros(){
  step_stride = nh_.param<int>("visualization/step_stride", 1);
  step_stride = step_stride < 1 ? 1 : (size_t)std::ceil(step_stride);

  speed_up = nh_.param<double>("visualization/speed_up", 1);
  speed_up = speed_up < 1 ? 1 : speed_up;

  pause_between_rollout = nh_.param<double>("visualization/pause_between_rollout", 0.0);

  rollout_stride = nh_.param<int>("visualization/rollout_stride", 1);
  rollout_stride = rollout_stride == 0 ? 1 : rollout_stride;

  default_visualization = nh_.param<bool>("visualization/default_visualization", false);
  ROS_INFO_STREAM("Visualization step stride: " << step_stride << std::endl
               << "Visualization speed up:    " << speed_up << std::endl
               << "Visualization pause roll:  " << pause_between_rollout << std::endl
               << "Visualization stride roll: " << rollout_stride << std::endl
               << "Visualization default mode:" << default_visualization << std::endl);
  initialized = true;
}
}
