/*!
 * @file     mppi_visualizer_ros.h
 * @author   Giuseppe Rizzi
 * @date     23.07.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <mppi/visualization/path_integral_visualizer.h>
#include "mppi_ros/RolloutsInfo.h"

namespace mppi_ros{

class PIVisualizerRos : public mppi::PathIntegralVisualizer{
 public:
  PIVisualizerRos(mppi::DynamicsBase::dynamics_ptr dynamics,
                  mppi::CostBase::cost_ptr cost,
                  mppi::GaussianSampler::sampler_ptr sampler,
                  const mppi::SolverConfig& config,
                  ros::NodeHandle& nh):
    PathIntegralVisualizer(dynamics, cost, sampler, config),
    nh_(nh){
    init_from_ros();

    rollouts_info_publisher = nh_.advertise<mppi_ros::RolloutsInfo>("/rollouts_info", 10);
    trigger_default_view_sub = nh_.subscribe("/set_default_view", 10, &PIVisualizerRos::set_default_view, this);

    rollouts_info_ros.weights.resize(config_.rollouts);
    rollouts_info_ros.cost_to_go.resize(config_.rollouts);
  };

 public:
  void publish_ros();

 private:
  void set_default_view(const std_msgs::BoolConstPtr& msg);

  void init_from_ros();


 private:
  ros::NodeHandle nh_;

  ros::Publisher weights_publisher;
  ros::Publisher rollouts_info_publisher;
  ros::Subscriber trigger_default_view_sub;

  mppi_ros::RolloutsInfo rollouts_info_ros;
};

}
