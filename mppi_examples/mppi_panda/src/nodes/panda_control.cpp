/*!
 * @file     panda_control.cpp
 * @author   Giuseppe Rizzi
 * @date     11.06.2020
 * @version  1.0
 * @brief    description
 */
#include "mppi_panda/controller_interface.h"

#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace panda;

int main(int argc, char** argv){
  // ros interface
  ros::init(argc, argv, "panda_control_node");
  ros::NodeHandle nh("~");
  auto controller = PandaControllerInterface(nh);

  bool kinematic_simulation = nh.param<bool>("dynamics/kinematic_simulation", true);
  bool raisim_backend = nh.param<bool>("raisim_backend", false);

  std::string robot_description = nh.param<std::string>("/robot_description", "");
  std::string robot_description_raisim = nh.param<std::string>("/robot_description_raisim", "");

  mppi::DynamicsBase::dynamics_ptr simulation;
  if (raisim_backend)
    simulation = std::make_shared<PandaRaisimDynamics>(robot_description_raisim, 0.01);
  else
    simulation = std::make_shared<PandaDynamics>(robot_description, kinematic_simulation);

  Eigen::VectorXd x = Eigen::VectorXd::Zero(PandaDim::STATE_DIMENSION);
  auto initial_configuration = nh.param<std::vector<double>>("initial_configuration", {});
  for(size_t i=0; i<initial_configuration.size(); i++)
    x(i) = initial_configuration[i];
  simulation->reset(x);

  mppi::DynamicsBase::input_t u;
  u = simulation->get_zero_input(x);

  ros::Publisher state_publisher = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
  sensor_msgs::JointState joint_state;
  joint_state.name = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5",
                      "panda_joint6", "panda_joint7"};
  joint_state.position.resize(7);
  joint_state.header.frame_id = "world";

  ros::Publisher ee_publisher = nh.advertise<geometry_msgs::PoseStamped>("/end_effector", 10);
  geometry_msgs::PoseStamped ee_pose;

  bool static_optimization = nh.param<bool>("static_optimization", false);
  double sim_dt = nh.param<double>("sim_dt", 0.01);

  u << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  // sim loop
  double sim_time = 0.0;
  controller.start();
  while(ros::ok()){
    auto start = std::chrono::steady_clock::now();
//    controller.set_observation(x, sim_time);
//    controller.get_input(x, u, sim_time);

    if (!static_optimization){
      x = simulation->step(u, sim_dt);
      sim_time += sim_dt;
    }

    for(size_t i=0; i<7; i++) joint_state.position[i] = x(i);
    joint_state.header.stamp = ros::Time::now();
    state_publisher.publish(joint_state);

    ee_pose = controller.get_pose_end_effector_ros(x);
    ee_publisher.publish(ee_pose);

    auto end = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()*1000;
    if (sim_dt - elapsed >0)
      ros::Duration(sim_dt - elapsed).sleep();

    ros::spinOnce();
  }
}