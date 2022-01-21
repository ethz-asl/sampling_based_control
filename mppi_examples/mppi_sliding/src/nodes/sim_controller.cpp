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

using namespace std::chrono;
using namespace manipulation;

mppi::observation_t x;
mppi::input_t u ;
double sim_time;
bool state_received_ = false;
void state_callback(const manipulation_msgs::StateConstPtr& state_msg)
{
  // modify the msgToEigen to adapt to panda
  ROS_INFO_STREAM(" position msg before conversion: ");
  for ( int i = 0 ; i < state_msg->arm_state.position.size(); i ++)
  {
    ROS_INFO_STREAM(state_msg->arm_state.position[i]);
  }
  ROS_INFO_STREAM("Time is now: " << sim_time);
  double temp_time;
  manipulation::conversions::msgToEigen_panda(*state_msg, x, temp_time);
  ROS_INFO_STREAM(" position info after conversion: " << x.transpose());

  state_received_ = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "manipulation_control_sim_node");
  ros::NodeHandle nh("~");
  std::string experiment_name;
  nh.param<std::string>("experiment_name", experiment_name, "test");

  ros::Subscriber state_subscriber_;
  std::string state_topic_;

  state_topic_ = "/observer/state";
  state_subscriber_ = nh.subscribe(state_topic_, 1, state_callback);

  // init logger
  signal_logger::setSignalLoggerStd();
  signal_logger::SignalLoggerOptions silo_options;
  silo_options.maxLoggingTime_ = 60.0;
  signal_logger::logger->initLogger(silo_options);

  // ros interface
  auto controller = std::make_unique<PandaControllerInterface>(nh);  // constructors are empty 

  observation_t x_nom;
  manipulation_msgs::State x_nom_ros;
  ros::Publisher x_nom_publisher_ =
      nh.advertise<manipulation_msgs::State>("/observer/state", 10);

  // init state and input
  x.setZero(STATE_DIMENSION);
  u.setZero(INPUT_DIMENSION);
  // mppi::observation_t x = simulation->get_state();  // get_state() returns x
  // mppi::input_t u = simulation->get_zero_input(x);

  ROS_INFO_STREAM("state and input inited");
  ROS_INFO_STREAM("init state: " << x.transpose());

  // init the controller
  sim_time = 0.0;
  bool ok = controller->init();
  if (!ok) throw std::runtime_error("Failed to initialize controller!");

  ROS_INFO_STREAM("controller inited");

  // set the very first observation
  // controller.set_observation(x, sim_time);
  // ROS_INFO_STREAM("init first observation as : " << x.transpose());

  // start controller
  bool sequential;
  nh.param<bool>("sequential", sequential, false);
  ROS_INFO_STREAM("Running in sequential mode? " << sequential);

  if (!sequential) controller->start();

  ROS_INFO_STREAM( "controller started" );

  // do some timing
  double elapsed;
  std::chrono::time_point<std::chrono::steady_clock> start, end;

  ros::Rate loop_rate(10);
  while (ros::ok()) {

    start = std::chrono::steady_clock::now();

    if(!state_received_)
    {
      ROS_INFO("state not received");
    } 

    if(state_received_)
    {

      controller->set_observation(x,sim_time);
      controller->update_reference(x, sim_time);

      //ROS_INFO_STREAM( "controller update , input is: "<< u.transpose());

      ROS_INFO_STREAM(" observation is: "<< x.transpose());

      controller->get_input_state(x, x_nom, u, sim_time);

      manipulation::conversions::eigenToMsg_panda(x_nom, sim_time, x_nom_ros);

    }  
    ros::spinOnce();
    loop_rate.sleep();
    
    sim_time = sim_time + 0.1;
  }

}

