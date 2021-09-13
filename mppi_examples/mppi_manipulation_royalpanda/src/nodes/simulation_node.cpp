//
// Created by giuseppe on 16.08.21.
//

#include <controller_manager/controller_manager.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <chrono>
#include <signal_logger/signal_logger.hpp>
#include <thread>
#include <rosgraph_msgs/Clock.h>
#include "manipulation_msgs/StateRequest.h"
#include "manipulation_msgs/conversions.h"
#include "mppi_manipulation_royalpanda/simulation.h"

using namespace std::chrono;
using namespace manipulation_royalpanda;

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulation");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // start logging
  signal_logger::setSignalLoggerStd();
  signal_logger::SignalLoggerOptions silo_options;
  silo_options.maxLoggingTime_ = 0.0;  // exponentially growing buffer
  signal_logger::logger->initLogger(silo_options);

  // do not advance until this time.
  // this is to make sure that all metrics collected by this time on
  // are comparable as in the previous time some controller switching
  // might still happen in the background
  double pause_time;
  if (!nh_private.getParam("pause_time", pause_time)) {
    ROS_ERROR("Failed to parse pause_time");
    return 0;
  }

  double max_sim_time;
  if (!nh_private.getParam("max_sim_time", max_sim_time)) {
    ROS_ERROR("Failed to parse max_sim_time");
    return 0;
  }

  std::string experiment_name;
  if (!nh_private.getParam("experiment_name", experiment_name)) {
    ROS_ERROR("Failed to parse experiment_name");
    return 0;
  }

  ROS_INFO_STREAM("The simulation will not advance until t=" << pause_time);

  // tell ROS to use the sim time
  nh.setParam("/use_sim_time", true);
  rosgraph_msgs::Clock ros_time;
  ros::Publisher clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

  RoyalPandaSim sim(nh_private);
  if (!sim.init_sim()) {
    ROS_ERROR("Failed to init the simulation.");
    return 0;
  }

  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);
  controller_manager::ControllerManager controller_manager(&sim, nh);
  ros::AsyncSpinner spinner(4, &queue);
  spinner.start();

  double t = 0.0;
  double dt = sim.get_time_step();
  ros::Duration period = ros::Duration().fromSec(dt);
  ros::Time curr_time = ros::Time().fromSec(t);
  time_point<steady_clock> start, end;

  double elapsed, remaining;

  manipulation_msgs::StateRequestRequest req;
  manipulation_msgs::StateRequestResponse res;

  signal_logger::add(t, "sim_time");
  signal_logger::logger->updateLogger();
  signal_logger::logger->startLogger();

  while (ros::ok() && t <= max_sim_time) {
    start = steady_clock::now();
    ROS_DEBUG_STREAM("Sim state:" << std::endl
                                  << std::setprecision(2)
                                  << manipulation::conversions::eigenToString(
                                         sim.get_state()));

    curr_time = ros::Time().fromSec(t);
    sim.read_sim(curr_time, period);

    //    if (enforce_determinism){
    //      // the server returns successfully only once it "builds" a sync
    //      state
    //      // at the requested timestamp
    //      // we need to keep calling read_sim so that the sim keeps publishing
    //      // the most recent state
    //      req.time = curr_time.toSec();
    //      while (!state_client.call(req, res) && ros::ok()) {
    //        partial1 = steady_clock::now();
    //        sim.read_sim(curr_time, period);
    //        partial2 = steady_clock::now();
    //        ros::Duration(0.001).sleep();
    //        ros::spinOnce();
    //      }
    //
    //      Eigen::VectorXd x;
    //      double t_req;
    //      manipulation::conversions::msgToEigen(res.state, x, t_req);
    //      ROS_DEBUG_STREAM("Req state:"
    //                       << std::endl
    //                       << std::setprecision(2)
    //                       << manipulation::conversions::eigenToString(x));
    //    }

    // update the controller input
    controller_manager.update(curr_time, period);

    // send it to the simulation
    if (t >= pause_time) {
      ROS_INFO_ONCE("Unpausing simulation!");
      sim.advance_sim(curr_time, period);
    }
    t += dt;

    // publish data to ros
    sim.publish_ros();
    ros_time.clock.fromSec(t);
    clock_publisher.publish(ros_time);

    // timing
    end = steady_clock::now();
    elapsed = (duration_cast<microseconds>(end - start).count() / 1e6);
    remaining = std::max(0.0, dt - elapsed);
    if (remaining > 0.0)
      std::this_thread::sleep_for(microseconds((int)(remaining * 1e6)));

    ros::spinOnce();
  }

  // save logs
  std::cout << "Saving logged data..." << std::endl;
  std::string file_path = __FILE__;
  std::string dir_path = file_path.substr(0, file_path.rfind("/"));
  file_path = dir_path + "/../../logs/" + experiment_name;
  signal_logger::logger->stopLogger();
  signal_logger::logger->saveLoggerData({signal_logger::LogFileType::BINARY},
                                        file_path);
  signal_logger::logger->cleanup();
  std::cout << "Done." << std::endl;

  spinner.stop();

  return 0;
}
