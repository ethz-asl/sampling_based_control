/*!
 * @file     panda_control.cpp
 * @author   Giuseppe Rizzi
 * @date     11.06.2020
 * @version  1.0
 * @brief    description
 */
#include "mppi_manipulation/controller_interface.h"

#include <manipulation_msgs/conversions.h>
#include <mppi_manipulation/dynamics_ros.h>
#include <mppi_manipulation/manipulation_safety_filter.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <chrono>
#include <signal_logger/signal_logger.hpp>

using namespace manipulation;

int main(int argc, char** argv) {
  ros::init(argc, argv, "panda_raisim_control_node");
  ros::NodeHandle nh("~");

  // init logger
  signal_logger::setSignalLoggerStd();
  signal_logger::SignalLoggerOptions silo_options;
  silo_options.maxLoggingTime_ = 60.0;
  signal_logger::logger->initLogger(silo_options);

  // ros interface
  auto controller = PandaControllerInterface(nh);

  auto robot_description = nh.param<std::string>("/robot_description", "");
  auto robot_description_raisim =
      nh.param<std::string>("/robot_description_raisim", "");
  auto object_description_raisim =
      nh.param<std::string>("/object_description_raisim", "");


  // Safety filter
  bool apply_safety_filter;
  nh.param<bool>("apply_safety_filter", apply_safety_filter, false);

  PandaMobileSafetyFilterSettings filter_settings;
  if (!filter_settings.init_from_ros(nh) && apply_safety_filter) {
    ROS_ERROR("Failed to initialize the safety filter!");
    return 0;
  }

  std::string robot_description_sf;
  if (!nh.param<std::string>("/robot_description_safety_filter",
                             robot_description_sf, "")) {
    ROS_ERROR("No /robot_description_safety_filter found on the param server.");
    return 0;
  }

  auto filter = std::make_unique<PandaMobileSafetyFilter>(robot_description_sf,
                                                          filter_settings);

  auto simulation = std::make_shared<ManipulatorDynamicsRos>(
      nh, robot_description_raisim, object_description_raisim, 0.015,
      PandaRaisimGains(), filter);

  // set initial state (which is also equal to the one to be tracked)
  // the object position and velocity is already set to 0
  observation_t x = observation_t::Zero(simulation->get_state_dimension());
  auto x0 = nh.param<std::vector<double>>("initial_configuration", {});
  for (size_t i = 0; i < x0.size(); i++) x(i) = x0[i];

  observation_t x_nom;
  manipulation_msgs::State x_nom_ros;
  ros::Publisher x_nom_publisher_ =
      nh.advertise<manipulation_msgs::State>("/observer/state", 10);

  ROS_INFO_STREAM("Resetting initial state to " << x.transpose());
  simulation->reset(x);

  // init control input
  mppi::input_t u;
  u = simulation->get_zero_input(x);

  bool static_optimization = nh.param<bool>("static_optimization", false);
  auto sim_dt = nh.param<double>("sim_dt", 0.01);
  double sim_time = 0.0;

  // init the controller
  bool ok = controller.init();
  if (!ok) throw std::runtime_error("Failed to initialize controller!");

  // set the very first observation
  controller.set_observation(x, sim_time);

  // start controller
  bool sequential;
  nh.param<bool>("sequential", sequential, false);
  if (!sequential) controller.start();

  // do some timing
  double elapsed;
  std::chrono::time_point<std::chrono::steady_clock> start, end;

  signal_logger::add(sim_time, "sim_time");
  signal_logger::add(u, "input");
  signal_logger::logger->updateLogger();

  signal_logger::logger->startLogger();

  while (ros::ok()) {
    start = std::chrono::steady_clock::now();
    controller.update_reference(sim_time);
    if (sequential) {
      controller.set_observation(x, sim_time);
      controller.update_policy();
      controller.get_input(x, u, sim_time);
      controller.publish_ros_default();
      controller.publish_ros();
    } else {
      controller.set_observation(x, sim_time);
      controller.get_input(x, u, sim_time);
    }

    if (!static_optimization) {
      x = simulation->step(u, sim_dt);
      sim_time += sim_dt;
    }

    controller.get_input_state(x, x_nom, u, sim_time);
    manipulation::conversions::eigenToMsg(x_nom, x_nom_ros);
    x_nom_publisher_.publish(x_nom_ros);

    end = std::chrono::steady_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
                  .count() /
              1000.0;
    if (sim_dt - elapsed > 0)
      ros::Duration(sim_dt - elapsed).sleep();
    else
      ROS_INFO_STREAM_THROTTLE(
          3.0, "Slower than real-time: " << elapsed / sim_dt << "x slower.");

    simulation->publish_ros();

    signal_logger::logger->collectLoggerData();
    ros::spinOnce();
  }

  std::cout << "Saving logged data..." << std::endl;
  signal_logger::logger->stopAndSaveLoggerData({signal_logger::LogFileType::BINARY});
  signal_logger::logger->cleanup();
  std::cout << "Done." << std::endl;
}