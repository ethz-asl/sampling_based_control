//
// Created by giuseppe on 16.08.21.
//

#include <controller_manager/controller_manager.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <chrono>
#include <thread>

#include "mppi_manipulation_royalpanda/simulation.h"

using namespace std::chrono;
using namespace manipulation_royalpanda;

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulation");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);

  RoyalPandaSim sim(nh_private);
  if (!sim.init_sim()) {
    ROS_ERROR("Failed to init the simulation.");
    return 0;
  }

  controller_manager::ControllerManager controller_manager(&sim, nh);
  ros::AsyncSpinner spinner(4, &queue);
  spinner.start();

  double t = 0.0;
  double dt = sim.get_time_step();
  ros::Duration period = ros::Duration().fromSec(dt);
  ros::Time curr_time = ros::Time().fromSec(t);

  time_point<steady_clock> start, end;
  double elapsed, remaining;

  while (ros::ok()) {
    start = steady_clock::now();

    // read variables from simulation and publish over ros
    sim.read_sim(curr_time, period);

    // update the controller input
    controller_manager.update(curr_time, period);

    // send it to the simulation
    sim.write_sim(curr_time, period);

    // publish data to ros
    sim.publish_ros();

    // advance time
    t += dt;
    curr_time = ros::Time().fromSec(t);

    // timing
    end = steady_clock::now();
    elapsed = (duration_cast<microseconds>(end - start).count() / 1e6);
    remaining = std::max(0.0, dt - elapsed);
    if (remaining > 0.0)
      std::this_thread::sleep_for(microseconds((int)(remaining * 1e6)));
  }

  spinner.stop();

  return 0;
}
