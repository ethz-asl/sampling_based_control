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
#include <mppi_panda_mobile/safety_filter/safety_filter.hpp>

using namespace manipulation;

int main(int argc, char** argv) {
  ros::init(argc, argv, "panda_raisim_control_node");
  ros::NodeHandle nh("~");

  // ros interface
  auto controller = PandaControllerInterface(nh);

  auto robot_description = nh.param<std::string>("/robot_description", "");
  auto robot_description_raisim =
      nh.param<std::string>("/robot_description_raisim", "");
  auto object_description_raisim =
      nh.param<std::string>("/object_description_raisim_sim", "");

  // Fixed base option
  bool fixed_base;
  if (!nh.param<bool>("fixed_base", fixed_base, false)) {
    ROS_ERROR_STREAM("Failed to find param fixed_base");
    return -1;
  }
  auto simulation = std::make_shared<ManipulatorDynamicsRos>(
      nh, robot_description_raisim, object_description_raisim, 0.015,
      fixed_base);

  // Safety filter
  bool apply_safety_filter;
  nh.param<bool>("apply_safety_filter", apply_safety_filter, false);

  if (apply_safety_filter && fixed_base) {
    ROS_ERROR("Safety filter only support moving base for the moment being.");
    return 0;
  }

  panda_mobile::PandaMobileSafetyFilterSettings filter_settings;
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

  panda_mobile::PandaMobileSafetyFilter filter(robot_description_sf,
                                               filter_settings);
  Eigen::VectorXd x_f, u_f, u_opt;
  Eigen::VectorXd torque_ext;
  x_f.setZero(10);  // optimizing only base and arm velocities
  u_f.setZero(10);
  u_opt.setZero(10);

  std_msgs::Float64 tank_energy;
  ros::Publisher tank_energy_publisher =
      nh.advertise<std_msgs::Float64>("/tank_energy", 1);

  // set initial state (which is also equal to the one to be tracked)
  // the object position and velocity is already set to 0
  observation_t x = observation_t::Zero(simulation->get_state_dimension());
  auto x0 = nh.param<std::vector<double>>("initial_configuration", {});
  for (size_t i = 0; i < x0.size(); i++) x(i) = x0[i];

  observation_t x_nom;
  //  manipulation_msgs::State x_nom_ros;
  //  ros::Publisher x_nom_publisher_ =
  //      nh.advertise<manipulation_msgs::State>("/observer/state", 10);
  std_msgs::Float32MultiArray x_nom_arr;
  x_nom_arr.data.resize(simulation->get_state_dimension());
  ros::Publisher x_nom_publisher_ =
      nh.advertise<std_msgs::Float32MultiArray>("/observer/nominal_state", 10);
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

  while (ros::ok()) {
    start = std::chrono::steady_clock::now();
    if (sequential) {
      controller.update_reference();
      controller.set_observation(x, sim_time);
      controller.update_policy();
      controller.get_input(x, u, sim_time);
      controller.get_input_state(x, x_nom, u, sim_time);
      controller.publish_ros_default();
      controller.publish_ros();
    } else {
      controller.set_observation(x, sim_time);
      controller.get_input(x, u, sim_time);
      controller.get_input_state(x, x_nom, u, sim_time);
    }

    // TODO(giuseppe) see how to integrate sf with accelerations
    //    if (apply_safety_filter) {
    //      x_f = x.head<10>();
    //      u_f = x_nom.tail<12>().head<10>();
    //      simulation->get_external_torque(torque_ext);
    //      filter.passivity_constraint()->update_passivity_constraint(
    //          torque_ext.head<10>());
    //      filter.apply(x_f, u_f, u_opt);
    //      x.tail<12>().head<10>() = u_opt;
    //      filter.passivity_constraint()->integrate_tank(u_opt.head<10>());
    //      tank_energy.data = filter.passivity_constraint()->get_tank_energy();
    //      tank_energy_publisher.publish(tank_energy);
    //    }

    if (!static_optimization) {
      x = simulation->step(u, sim_dt);
      sim_time += sim_dt;
    }

    controller.get_input_state(x, x_nom, u, sim_time);

    // TODO(giuseppe) need to change conversions if using accelerations
    // manipulation::conversions::eigenToMsg(x_nom, x_nom_ros);
    // x_nom_publisher_.publish(x_nom_ros);
    for (int i = 0; i < x_nom.size(); i++) x_nom_arr.data[i] = x_nom[i];
    x_nom_publisher_.publish(x_nom_arr);

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
    ros::spinOnce();
  }
}