/*!
 * @file    omav_velocity_control.cpp
 * @author  Matthias Studiger
 * @date    10.04.2021
 * @version 1.0
 * @brief   description
 */
#include "mppi_omav_velocity/omav_velocity_control.h"

using namespace omav_velocity;

OmavTrajectoryGenerator::OmavTrajectoryGenerator(
    const ros::NodeHandle &nh, const ros::NodeHandle &private_nh)
    : nh_(nh), private_nh_(private_nh) {
  initializePublishers();
  initializeSubscribers();
}

OmavTrajectoryGenerator::~OmavTrajectoryGenerator() {}

void OmavTrajectoryGenerator::initializeSubscribers() {
  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                &OmavTrajectoryGenerator::odometryCallback,
                                this, ros::TransportHints().tcpNoDelay());
  ROS_INFO_STREAM("Subscriber initialized");
  odometry_bool_ = true;
}

void OmavTrajectoryGenerator::initializePublishers() {
  reference_publisher_ =
      nh_.advertise<geometry_msgs::PoseStamped>("/mppi_pose_desired", 1);

  indicator_publisher_ = nh_.advertise<std_msgs::Int64>("/indicator", 1);

  take_off_srv_ = nh_.advertiseService(
      "take_off", &OmavTrajectoryGenerator::takeOffSrv, this);

  execute_trajectory_srv_ = nh_.advertiseService(
      "go_to_goal", &OmavTrajectoryGenerator::executeTrajectorySrv, this);

  homing_srv_ =
      nh_.advertiseService("homing", &OmavTrajectoryGenerator::homingSrv, this);
  landing_srv_ =
      nh_.advertiseService("land", &OmavTrajectoryGenerator::landingSrv, this);
}

void OmavTrajectoryGenerator::odometryCallback(
    const nav_msgs::OdometryConstPtr &odometry_msg) {
  mav_msgs::eigenOdometryFromMsg(*odometry_msg, &current_odometry_);
  ROS_INFO_ONCE("MPPI got first odometry message");
  odometry_bool_ = false;
}

bool OmavTrajectoryGenerator::takeOffSrv(std_srvs::Empty::Request &request,
                                         std_srvs::Empty::Response &response) {
  geometry_msgs::PoseStamped take_off_pose_msg;
  Eigen::VectorXd take_off_pose(7);
  take_off_pose << 0, 0, 1, 1, 0, 0, 0;
  omav_velocity::conversions::PoseMsgFromVector(take_off_pose,
                                                take_off_pose_msg);
  reference_publisher_.publish(take_off_pose_msg);
  indicator.data = 0;
  indicator_publisher_.publish(indicator);
  return true;
}

bool OmavTrajectoryGenerator::executeTrajectorySrv(
    std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  geometry_msgs::PoseStamped goal_pose_msg;
  Eigen::VectorXd goal_pose(7);
  goal_pose << 10.0, 10.0, 10.0, 1.0, 0.0, 0.0, 0.0;
  omav_velocity::conversions::PoseMsgFromVector(goal_pose, goal_pose_msg);
  reference_publisher_.publish(goal_pose_msg);
  indicator.data = 1;
  indicator_publisher_.publish(indicator);
  return true;
}

bool OmavTrajectoryGenerator::homingSrv(std_srvs::Empty::Request &request,
                                        std_srvs::Empty::Response &response) {
  geometry_msgs::PoseStamped home_pose_msg;
  Eigen::VectorXd home_pose(7);
  home_pose << 0, 0, 1, 1, 0, 0, 0;
  omav_velocity::conversions::PoseMsgFromVector(home_pose, home_pose_msg);
  reference_publisher_.publish(home_pose_msg);
  indicator.data = 1;
  indicator_publisher_.publish(indicator);
  return true;
}

bool OmavTrajectoryGenerator::landingSrv(std_srvs::Empty::Request &request,
                                         std_srvs::Empty::Response &response) {
  geometry_msgs::PoseStamped landing_pose_msg;
  omav_velocity::conversions::PoseMsgFromVector(start_position_,
                                                landing_pose_msg);
  reference_publisher_.publish(landing_pose_msg);
  indicator.data = 0;
  indicator_publisher_.publish(indicator);
}

void OmavTrajectoryGenerator::get_odometry(observation_t &x) {
  x.head<3>() = current_odometry_.position_W;
  x(3) = current_odometry_.orientation_W_B.w();
  x.segment<3>(4) = current_odometry_.orientation_W_B.vec();
  x.segment<3>(7) = current_odometry_.getVelocityWorld();
  x.tail<3>() = current_odometry_.angular_velocity_B;
}

void OmavTrajectoryGenerator::get_start_position(observation_t &x) {
  start_position_ = x;
}

int main(int argc, char **argv) {
  // Initialize Ros Node
  ros::init(argc, argv, "omav_control_node");
  ros::NodeHandle nh("~"), nh_public;

  std::shared_ptr<omav_velocity::OmavTrajectoryGenerator> omav_trajectory_node(
      new omav_velocity::OmavTrajectoryGenerator(nh_public, nh));

  // ros interface
  auto controller = OMAVControllerInterface(nh, nh_public);
  ROS_INFO_STREAM("Controller Created");

  auto robot_description_raisim =
      nh.param<std::string>("/robot_description_raisim", "");
  ROS_INFO_STREAM("Robot Description Raisim Loaded");
  // set initial state
  observation_t x = observation_t::Zero(13);
  auto x0 = nh.param<std::vector<double>>("initial_configuration", {});
  for (size_t i = 0; i < x0.size(); i++)
    x(i) = x0[i];
  // Set nominal state
  observation_t x_nom = observation_t::Zero(13);

  // init control input
  mppi::DynamicsBase::input_t u = input_t::Zero(6);

  bool static_optimization = nh.param<bool>("static_optimization", false);
  auto sim_dt = nh.param<double>("sim_dt", 0.015);
  double sim_time = 0.0;

  // init the controller
  bool ok = controller.init();
  if (!ok)
    throw std::runtime_error("Failed to initialize controller");

  while (omav_trajectory_node->odometry_bool_) {
    ROS_INFO_STREAM_ONCE("No Odometry recieved yet");
    ros::spinOnce();
  }
  ROS_INFO_STREAM("First Odometry recieved");
  // Set fir odomety value as reference
  omav_trajectory_node->get_odometry(x);
  omav_trajectory_node->get_start_position(x);
  controller.set_reference(x);

  // start controller
  bool sequential;
  nh.param<bool>("sequential", sequential, false);
  if (!sequential)
    controller.start();

  // do some timing
  double elapsed;

  while (ros::ok()) {
    if (sequential) {
      controller.update_reference();
      controller.set_observation(x, sim_time);
      controller.update_policy();
      controller.get_input_state(x, x_nom, u, sim_time);
      controller.publish_ros_default();
      // Additional publisher for additional visualization
      // controller.publish_optimal_rollout();
      // controller.publish_trajectories();

    } else {
      omav_trajectory_node->get_odometry(x);
      controller.set_observation(x, sim_time);
      controller.get_input_state(x, x_nom, u, sim_time);
      // controller.publish_optimal_rollout();
    }

    ros::spinOnce();
  }
}
