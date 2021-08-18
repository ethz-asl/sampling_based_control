//
// Created by giuseppe on 16.08.21.
//

#include <controller_manager/controller_manager.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <chrono>
#include <thread>

#include "manipulation_msgs/StateRequest.h"
#include "manipulation_msgs/conversions.h"
#include "mppi_manipulation_royalpanda/simulation.h"

using namespace std::chrono;
using namespace manipulation_royalpanda;

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulation");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // when enforcing determinism we make sure (through a service call)
  // that the latest state from simulation has been collected as
  // on the real hardware
  bool enforce_determinism;
  if(!nh_private.getParam("enforce_determinism", enforce_determinism)){
    ROS_ERROR("Failed to parse enforce_determinism");
    return 0;
  }

  ros::ServiceClient state_client =
      nh.serviceClient<manipulation_msgs::StateRequest>(
          "/observer/state_request");
  if (!state_client.waitForExistence(ros::Duration(3))) {
    ROS_ERROR("Service /observer/state_request does not exist.");
    return 0;
  }


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

  while (ros::ok()) {
    start = steady_clock::now();
    ROS_DEBUG_STREAM("Sim state=[" << std::setprecision(2) << sim.get_state().transpose() << "]");

    if (enforce_determinism){
      // the server returns successfully only once it "builds" a sync state
      // at the requested timestamp
      // we need to keep calling read_sim so that the sim keeps publishing
      // the most recent state
      req.time = curr_time.toSec();
      while(!state_client.call(req, res)) {
        sim.read_sim(curr_time, period);
        ros::Duration(0.001).sleep();
        ros::spinOnce();
      }

      Eigen::VectorXd x;
      double t_req;
      manipulation::conversions::msgToEigen(res.state, x, t_req);
      ROS_DEBUG_STREAM("Req state=[" << std::setprecision(2) << x.transpose() << "]");
    }


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

    // process all general callbacks
    ros::spinOnce();
  }

  spinner.stop();

  return 0;
}
