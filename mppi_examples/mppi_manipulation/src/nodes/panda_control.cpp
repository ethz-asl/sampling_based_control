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
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <chrono>
#include <signal_logger/signal_logger.hpp>

using namespace std::chrono;
using namespace manipulation;

int main(int argc, char** argv) {
  ros::init(argc, argv, "panda_raisim_control_node");
  ros::NodeHandle nh("~");

  std::string experiment_name;
  nh.param<std::string>("experiment_name", experiment_name, "test");

  // init logger
  signal_logger::setSignalLoggerStd();
  signal_logger::SignalLoggerOptions silo_options;
  silo_options.maxLoggingTime_ = 60.0;
  signal_logger::logger->initLogger(silo_options);

  // ros interface
  auto controller = PandaControllerInterface(nh);

  bool is_sim = true;
  DynamicsParams dynamics_params;
  if (!dynamics_params.init_from_ros(nh, is_sim)) {
    ROS_ERROR("Failed to parse dynamics parameters");
    return 0;
  }
  ROS_INFO_STREAM(dynamics_params);
  auto simulation =
      std::make_shared<ManipulatorDynamicsRos>(nh, dynamics_params);

  observation_t x_nom;
  manipulation_msgs::State x_nom_ros;
  ros::Publisher x_nom_publisher_ =
      nh.advertise<manipulation_msgs::State>("/observer/state", 10);

  // init state and input
  mppi::observation_t x = simulation->get_state();
  mppi::input_t u = simulation->get_zero_input(x);

  // init the controller
  double sim_time = 0.0;
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

    x = simulation->step(u, simulation->get_dt());
    sim_time += simulation->get_dt();

    controller.get_input_state(x, x_nom, u, sim_time);
    manipulation::conversions::eigenToMsg(x_nom, x_nom_ros);
    x_nom_publisher_.publish(x_nom_ros);

    end = steady_clock::now();
    elapsed = duration_cast<milliseconds>(end - start).count() / 1000.0;
    if (simulation->get_dt() - elapsed > 0)
      ros::Duration(simulation->get_dt() - elapsed).sleep();
    else
      ROS_INFO_STREAM_THROTTLE(
          3.0, "Slower than real-time: " << elapsed / simulation->get_dt()
                                         << "x slower.");

    simulation->publish_ros();

    signal_logger::logger->collectLoggerData();
    ros::spinOnce();
  }

  // save logs
  std::cout << "Saving logged data..." << std::endl;
  std::string file_path = __FILE__;
  std::string dir_path = file_path.substr(0, file_path.rfind("/"));
  file_path = dir_path + "/../../data/logs/" + experiment_name;
  signal_logger::logger->stopLogger();
  signal_logger::logger->saveLoggerData({signal_logger::LogFileType::BINARY},
                                        file_path);
  signal_logger::logger->cleanup();
  std::cout << "Done." << std::endl;
}