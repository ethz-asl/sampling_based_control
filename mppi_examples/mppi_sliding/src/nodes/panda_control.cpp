/*!
 * @file     panda_control.cpp
 * @author   Boyang Sun
 * @date     20.10.2021
 * @version  1.0
 * @brief    description
 */
#include "mppi_sliding/controller_interface.h"

#include <manipulation_msgs/conversions.h>
#include <mppi_sliding/dynamics_ros.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <chrono>
#include <signal_logger/signal_logger.hpp>
#include <random>

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
  auto controller = PandaControllerInterface(nh);  // constructors are empty 

  bool is_sim = true;
  DynamicsParams dynamics_params;     // init params (dynamics_param.cpp)
  if (!dynamics_params.init_from_ros(nh, is_sim)) {
    ROS_ERROR("Failed to parse dynamics parameters");
    return 0;
  }
  ROS_INFO_STREAM(
      "Successfully parsed simulation dynamics parameter: " << dynamics_params);

  auto simulation =
      std::make_shared<ManipulatorDynamicsRos>(nh, dynamics_params);  // init sim dynamics (->dynamic_ros.cpp -> dynamic.cpp)

  ROS_INFO_STREAM("real world simulation initiated");

  observation_t x_nom;
  manipulation_msgs::State x_nom_ros;
  ros::Publisher x_nom_publisher_ =
      nh.advertise<manipulation_msgs::State>("/observer/state", 10);

  std_msgs::Float64 stage_cost;
  ros::Publisher stage_cost_publisher_ = 
      nh.advertise<std_msgs::Float64>("/stage_cost",10);

  // init state and input
  mppi::observation_t x = simulation->get_state();  // get_state() returns x
  mppi::input_t u = simulation->get_zero_input(x);

  ROS_INFO_STREAM("state and input inited");
  ROS_INFO_STREAM("init state: " << x.transpose());

  // init the controller
  double sim_time = 0.0;
  bool ok = controller.init();
  if (!ok) throw std::runtime_error("Failed to initialize controller!");

  ROS_INFO_STREAM("controller inited");

  // set the very first observation
  controller.set_observation(x, sim_time);
  ROS_INFO_STREAM("init first observation as : " << x.transpose());
  
  // start controller
  bool sequential;
  nh.param<bool>("sequential", sequential, false);
  ROS_INFO_STREAM("Running in sequential mode? " << sequential);

  if (!sequential) controller.start();

  ROS_INFO_STREAM( "controller started" );

  // do some timing
  double elapsed;
  std::chrono::time_point<std::chrono::steady_clock> start, end;

  signal_logger::add(sim_time, "sim_time");
  signal_logger::add(u, "input");
  signal_logger::logger->updateLogger();
  signal_logger::logger->startLogger();

  while (ros::ok()) {

    start = std::chrono::steady_clock::now();
    controller.update_reference(x, sim_time);

    if (sequential) {
      controller.set_observation(x, sim_time);
      controller.update_policy();
      controller.get_input(x, u, sim_time);
      controller.publish_ros_default();
      controller.publish_ros();
    }
    else {
      controller.set_observation(x, sim_time);
    }

    auto start_ = std::chrono::high_resolution_clock::now();

    x = simulation->step(u, simulation->get_dt());

    auto end_ = std::chrono::high_resolution_clock::now();
    auto tstamp = end_ - start_;
    int32_t sec = std::chrono::duration_cast<std::chrono::microseconds>(tstamp).count();
    //ROS_INFO_STREAM("duration of simulation step: " << sec << "microseconds");

    sim_time += simulation->get_dt();

    // compensate z-axis, since the frame origin of mug obj is at the bottom
    x(2*ARM_GRIPPER_DIM+2) += 0.05;

    //ROS_INFO_STREAM("raw x : " << x.transpose());

    // add noise
    for(int i = 0; i < 2; i ++)
    {
      std::random_device rd{};
      std::mt19937 gen{rd()};
      std::normal_distribution<> d{x(2*ARM_GRIPPER_DIM+i), 0.01};
      x(2*ARM_GRIPPER_DIM+i) = d(gen); 
    }

    //ROS_INFO_STREAM("noise x : " << x.transpose());    

    controller.get_input_state(x, x_nom, u, sim_time);

    manipulation::conversions::eigenToMsg_panda(x_nom, sim_time, x_nom_ros);
    x_nom_publisher_.publish(x_nom_ros);

    stage_cost.data = controller.get_stage_cost(x, u, sim_time);
    stage_cost_publisher_.publish(stage_cost);

    end = steady_clock::now();
    elapsed = duration_cast<milliseconds>(end - start).count() / 1000.0;

    if (simulation->get_dt() - elapsed > 0)
      ros::Duration(simulation->get_dt() - elapsed).sleep();
    else
      ROS_INFO_STREAM_THROTTLE(
          3.0, "Slower than real-time: " << elapsed / simulation->get_dt()
                                         << "x slower.");

    simulation->publish_ros();

    //signal_logger::logger->collectLoggerData();
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

