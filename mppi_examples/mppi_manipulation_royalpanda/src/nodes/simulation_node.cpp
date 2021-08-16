//
// Created by giuseppe on 16.08.21.
//

#include "mppi_manipulation_royalpanda/simulation.h"
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <chrono>
#include <thread>
#include <ros/callback_queue.h>

using namespace std::chrono;
using namespace manipulation_royalpanda;

int main(int argc, char** argv){
  ros::init(argc, argv, "simulation");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);

  RoyalPandaSim sim(nh_private);
  if (!sim.init_sim()){
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

  while (ros::ok()){
    start = steady_clock::now();

    sim.read_sim(curr_time, period);
    controller_manager.update(curr_time, period);
    sim.write_sim(curr_time, period);
    sim.publish_ros();
    t += dt;
    curr_time = ros::Time().fromSec(t);

    end = steady_clock::now();
    elapsed = (duration_cast<microseconds>(end-start).count() / 1e6);
    remaining = std::max(0.0, dt-elapsed);
    if (remaining > 0.0)
      std::this_thread::sleep_for(microseconds ((int)(remaining * 1e6)));
  }

 spinner.stop();

  return 0;
}
