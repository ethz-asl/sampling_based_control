/*!
 * @file     pendulum_cart_control.cpp
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_mobile_panda/cost.h"
#include <ros/ros.h>
#include "mppi_mobile_panda/dynamics.h"
#include "mppi_mobile_panda/ros_interface.h"

using namespace mobile_panda;

int main(int argc, char** argv){
  ros::init(argc, argv, "panda_free_node");
  ros::NodeHandle nh("~");
  auto ros_interface = PandaRosInterface(nh);

  auto config = PandaDynamicsConfig();
  config.substeps = nh.param<double>("dynamics/substeps", 1.0);

  auto panda_dynamics = PandaDynamics();
  panda_dynamics.set_dynamic_properties(config);

  double dt = 0.01;
  double cost;
  ros::Rate rate(1.0/dt);
  auto panda_cost = PandaCost();
  PandaDynamics::input_t u = PandaDynamics::input_t::Zero(PandaDim::INPUT_DIMENSION);
  u(7) = 0.5; // forward speed
  u(8) = 0.1; // angular speed
  pinocchio::SE3 end_effector_position;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(PandaDim::STATE_DIMENSION);
  auto initial_configuration = nh.param<std::vector<double>>("initial_configuration", {});
  for(size_t i=0; i<initial_configuration.size(); i++)
    x(i) = initial_configuration[i];
  panda_dynamics.reset(x);

  // wait for rviz to spawn
  ros::Duration(3.0).sleep();
  while (ros::ok()){
    x = panda_dynamics.step(u, dt);
    cost = panda_cost.get_stage_cost(x);
    end_effector_position = panda_cost.get_pose_end_effector(x);

    ros_interface.publish_state(x);
    ros_interface.publish_end_effector_pose(end_effector_position);
    ros_interface.publish_cost(cost);
    rate.sleep();
  }
}
