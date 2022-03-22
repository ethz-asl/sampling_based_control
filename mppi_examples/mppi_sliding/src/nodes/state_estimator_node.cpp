/*!
 * @file     state_estimator.cpp
 * @author   Boyang Sun
 * @date     21.03.2022
 * @version  1.0
 * @brief    A state estimation node, take prediction and observation, output estimated state
 */

#include <manipulation_msgs/conversions.h>
#include <mppi_sliding/dynamics_ros.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <chrono>
#include <signal_logger/signal_logger.hpp>
#include <random>
#include <visualization_msgs/Marker.h>

using namespace std::chrono;
using namespace manipulation;


mppi::input_t u;
mppi::observation_t x_observe;
double x_observe_t;

void inputCallback(const manipulation_msgs::Input::ConstPtr& input_ros)
{   
    manipulation::conversions::msgToEigen_panda(*input_ros, u);
}


void stateCallback(const manipulation_msgs::State::ConstPtr& state_ros)
{ 
  manipulation::conversions::msgToEigen_panda(*state_ros, x_observe, x_observe_t);  
  return;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "state_estimation_node");
  ros::NodeHandle nh("~");
  std::string experiment_name;
  bool add_noise;
  nh.param<std::string>("experiment_name", experiment_name, "test");
  

  DynamicsParams dynamics_params;     // init params (dynamics_param.cpp)
  if (!dynamics_params.init_from_ros(nh, true)) {
    ROS_ERROR("Failed to parse dynamics parameters");
    return 0;
  }
  ROS_INFO_STREAM(
      "Successfully parsed simulation dynamics parameter: " << dynamics_params);

  auto simulation =
      std::make_shared<PandaRaisimDynamics>(dynamics_params, false);  // init sim dynamics (->dynamic_ros.cpp -> dynamic.cpp)

  ROS_INFO_STREAM("real world simulation initiated");
  ros::Subscriber u_subscriber_ = 
    nh.subscribe<manipulation_msgs::Input>("/observer/panda_input", 1, inputCallback);

  ros::Subscriber x_subscriber_ = 
    nh.subscribe<manipulation_msgs::State>("/observer/state", 1, stateCallback);
    
   // init state
   mppi::observation_t x_sim = simulation->get_state();  // get_state() returns x
   mppi::observation_t x_fused = simulation->get_state();  // get_state() returns x
   x_observe = simulation->get_state(); 
   
   ROS_INFO_STREAM("init state: " << x_sim.transpose());
   
   manipulation_msgs::State x_est_ros;
   ros::Publisher x_est_publisher_ =
      nh.advertise<manipulation_msgs::State>("/estimator/state", 10);

   // init time
   double sim_time = 0.0;

   // do some timing
   double elapsed;
   std::chrono::time_point<std::chrono::steady_clock> start, end;

   // start with zero input
   u = simulation->get_zero_input(x_sim);


  ros::Publisher object_est_publisher_; 
  visualization_msgs::Marker object_estimated_marker_;
  object_est_publisher_ = nh.advertise<visualization_msgs::Marker>("/observer/estiamted_object", 10);
  object_estimated_marker_.type = visualization_msgs::Marker::CYLINDER;
  object_estimated_marker_.header.frame_id = "world";
  object_estimated_marker_.action = visualization_msgs::Marker::ADD;
  object_estimated_marker_.color.r = 0.0;
  object_estimated_marker_.color.b = 1.0;
  object_estimated_marker_.color.g = 0.0;
  object_estimated_marker_.color.a = 0.6;

  while (ros::ok()) {
    
    ROS_INFO_STREAM("Get the input: " << u.transpose());

    start = std::chrono::steady_clock::now();
    // auto start_ = std::chrono::high_resolution_clock::now();

    // // step the state estimation simulation
    // x_sim = simulation->step(u, simulation->get_dt());
    // simulation->reset(x_fused,simulation->get_dt());
    // simulation->set_control(u);
    simulation->advance();
    x_sim = simulation->get_state();
    x_fused.head<2*BASE_ARM_GRIPPER_DIM>() = x_observe.head<2*BASE_ARM_GRIPPER_DIM>(); // Panda state just take the observation
    x_fused.tail<STATE_DIMENSION - 2*BASE_ARM_GRIPPER_DIM >() =  x_observe.tail<STATE_DIMENSION - 2*BASE_ARM_GRIPPER_DIM >(); 
    simulation->reset(x_fused,simulation->get_dt());
    // x_sim = simulation->get_state();
    // simulation x and observation x Fusion
    

    ROS_INFO_STREAM("Get the obj state: " << x_fused.segment<OBJECT_DIMENSION>(2 * BASE_ARM_GRIPPER_DIM).transpose());
    // auto end_ = std::chrono::high_resolution_clock::now();
    // auto tstamp = end_ - start_;

    // int32_t sec = std::chrono::duration_cast<std::chrono::microseconds>(tstamp).count();

    // //ROS_INFO_STREAM("duration of simulation step: " << sec << "microseconds");

    sim_time += simulation->get_dt();

    manipulation::conversions::eigenToMsg_panda(x_sim, sim_time, x_est_ros);
    x_est_publisher_.publish(x_est_ros);

    object_estimated_marker_.scale.x = 2*x_sim[ 2 * BASE_ARM_GRIPPER_DIM + OBJECT_DIMENSION +4];
    object_estimated_marker_.scale.y = 2*x_sim[ 2 * BASE_ARM_GRIPPER_DIM + OBJECT_DIMENSION +4];
    object_estimated_marker_.scale.z = x_sim[ 2 * BASE_ARM_GRIPPER_DIM + OBJECT_DIMENSION +5];

    object_estimated_marker_.pose.position.x = x_sim[ 2 * BASE_ARM_GRIPPER_DIM ];
    object_estimated_marker_.pose.position.y = x_sim[ 2 * BASE_ARM_GRIPPER_DIM +1];
    object_estimated_marker_.pose.position.z = x_sim[ 2 * BASE_ARM_GRIPPER_DIM +2];
    object_estimated_marker_.pose.orientation.x = x_sim[ 2 * BASE_ARM_GRIPPER_DIM +3];
    object_estimated_marker_.pose.orientation.y = x_sim[ 2 * BASE_ARM_GRIPPER_DIM +4];
    object_estimated_marker_.pose.orientation.z = x_sim[ 2 * BASE_ARM_GRIPPER_DIM +5];
    object_estimated_marker_.pose.orientation.w = x_sim[ 2 * BASE_ARM_GRIPPER_DIM +6];
    object_estimated_marker_.id = 0;

    object_est_publisher_.publish(object_estimated_marker_);

    end = steady_clock::now();
    elapsed = duration_cast<microseconds>(end - start).count() / (1000.0*1000.0);
    ROS_INFO_STREAM("one loop takes: " << elapsed << " sec");

    if (simulation->get_dt() - elapsed > 0)
      ros::Duration(simulation->get_dt() - elapsed).sleep();
    else
      ROS_INFO_STREAM_THROTTLE(
          3.0, "Slower than real-time: " << elapsed / simulation->get_dt()
                                         << "x slower.");


    ros::spinOnce();
  }
  
}

