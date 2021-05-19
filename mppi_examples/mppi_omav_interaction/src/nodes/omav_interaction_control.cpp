/*!
 * @file    omav_velocity_control.cpp
 * @author  Matthias Studiger
 * @date    10.04.2021
 * @version 1.0
 * @brief   description
 */
#include "mppi_omav_interaction/omav_interaction_control.h"

using namespace omav_interaction;

OmavTrajectoryGenerator::OmavTrajectoryGenerator(
    const ros::NodeHandle &nh, const ros::NodeHandle &private_nh)
    : nh_(nh), private_nh_(private_nh),
      goal_param_server_(ros::NodeHandle(private_nh, "Goal_parameters")),
      cost_param_server_(ros::NodeHandle(private_nh, "Cost_parameters")) {
  initializePublishers();
  initializeSubscribers();
  // setup dynamic reconfigure
  dynamic_reconfigure::Server<
      mppi_omav_interaction::MPPIOmavGoalConfig>::CallbackType f;
  dynamic_reconfigure::Server<
      mppi_omav_interaction::MPPIOmavCostConfig>::CallbackType g;
  f = boost::bind(&OmavTrajectoryGenerator::GoalParamCallback, this, _1, _2);
  g = boost::bind(&OmavTrajectoryGenerator::CostParamCallback, this, _1, _2);
  goal_param_server_.setCallback(f);
  cost_param_server_.setCallback(g);
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

  take_off_srv_ = nh_.advertiseService(
      "take_off", &OmavTrajectoryGenerator::takeOffSrv, this);

  execute_trajectory_srv_ = nh_.advertiseService(
      "go_to_goal", &OmavTrajectoryGenerator::executeTrajectorySrv, this);

  homing_srv_ =
      nh_.advertiseService("homing", &OmavTrajectoryGenerator::homingSrv, this);
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
  omav_interaction::conversions::PoseStampedMsgFromVector(take_off_pose,
                                                          take_off_pose_msg);
  reference_publisher_.publish(take_off_pose_msg);
  return true;
}

bool OmavTrajectoryGenerator::executeTrajectorySrv(
    std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
  geometry_msgs::PoseStamped goal_pose_msg;
  Eigen::VectorXd goal_pose(7);
  goal_pose << 10.0, 10.0, 10.0, 1.0, 0.0, 0.0, 0.0;
  omav_interaction::conversions::PoseStampedMsgFromVector(goal_pose,
                                                          goal_pose_msg);
  reference_publisher_.publish(goal_pose_msg);
  return true;
}

bool OmavTrajectoryGenerator::homingSrv(std_srvs::Empty::Request &request,
                                        std_srvs::Empty::Response &response) {
  geometry_msgs::PoseStamped home_pose_msg;
  Eigen::VectorXd home_pose(7);
  home_pose << 0, 0, 1, 1, 0, 0, 0;
  omav_interaction::conversions::PoseStampedMsgFromVector(home_pose,
                                                          home_pose_msg);
  reference_publisher_.publish(home_pose_msg);
  return true;
}

void OmavTrajectoryGenerator::get_odometry(observation_t &x) {
  x.head<3>() = current_odometry_.position_W;
  x(3) = current_odometry_.orientation_W_B.w();
  x.segment<3>(4) = current_odometry_.orientation_W_B.vec();
  x.segment<3>(7) = current_odometry_.getVelocityWorld();
  x.segment<3>(10) = current_odometry_.angular_velocity_B;
}

void OmavTrajectoryGenerator::GoalParamCallback(
    mppi_omav_interaction::MPPIOmavGoalConfig &config, uint32_t level) {
  geometry_msgs::PoseStamped rqt_pose_msg;
  Eigen::VectorXd rqt_pose(7);
  rqt_pose << config.goal_pos_x, config.goal_pos_y, config.goal_pos_z, 1, 0, 0,
      0;
  omav_interaction::conversions::PoseStampedMsgFromVector(rqt_pose,
                                                          rqt_pose_msg);
  reference_publisher_.publish(rqt_pose_msg);
  if (config.reset_object) {
    reset_object_ = true;
  }
}

void OmavTrajectoryGenerator::CostParamCallback(
    mppi_omav_interaction::MPPIOmavCostConfig &config, uint32_t level) {
  // Update OMAV pose cost
  rqt_cost_.Q_distance_x = config.Q_x_omav;
  rqt_cost_.Q_distance_y = config.Q_y_omav;
  rqt_cost_.Q_distance_z = config.Q_z_omav;
  rqt_cost_.Q_orientation = config.Q_orientation_omav;
  rqt_cost_.pose_costs << rqt_cost_.Q_distance_x, rqt_cost_.Q_distance_y,
      rqt_cost_.Q_distance_z, rqt_cost_.Q_orientation, rqt_cost_.Q_orientation,
      rqt_cost_.Q_orientation;
  rqt_cost_.Q_pose = rqt_cost_.pose_costs.asDiagonal();
  // Update Object cost
  rqt_cost_.Q_object_x = config.Q_object;
  rqt_cost_.object_costs << rqt_cost_.Q_object_x, rqt_cost_.Q_object_x,
      rqt_cost_.Q_object_x, rqt_cost_.Q_object_x, rqt_cost_.Q_object_x,
      rqt_cost_.Q_object_x;
  rqt_cost_.Q_object = rqt_cost_.object_costs.asDiagonal();
  // Update velocity cost
  rqt_cost_.Q_lin_vel = config.Q_linear_velocity;
  rqt_cost_.vel_costs << rqt_cost_.Q_lin_vel, rqt_cost_.Q_lin_vel,
      rqt_cost_.Q_lin_vel, config.Q_roll, config.Q_pitch, config.Q_yaw;
  rqt_cost_.Q_vel = rqt_cost_.vel_costs.asDiagonal();
  // Update Handle Hook cost components
  rqt_cost_.handle_hook_thresh = config.Handle_Hook_Threshold;
  rqt_cost_.Q_handle_hook = config.Handle_Hook_Cost;
  // Update floor cost components
  rqt_cost_.floor_thresh = config.Floor_Threshold;
  rqt_cost_.Q_floor = config.floor_cost;
  rqt_cost_.Q_power = config.power_cost;
  rqt_cost_.Q_torque = config.torque_cost;
  std::cout << rqt_cost_.Q_vel << std::endl;
  rqt_cost_bool_ = true;
}

int main(int argc, char **argv) {
  // Initialize Ros Node
  ros::init(argc, argv, "omav_control_node");
  ros::NodeHandle nh("~"), nh_public;
  // Check if running with omav interface
  bool running_rotors = nh.param<bool>("running_rotors", false);

  std::shared_ptr<omav_interaction::OmavTrajectoryGenerator>
      omav_trajectory_node(
          new omav_interaction::OmavTrajectoryGenerator(nh_public, nh));
  // ros interface
  auto controller = OMAVControllerInterface(nh, nh_public);
  ROS_INFO_STREAM("Controller Created");

  auto robot_description_raisim =
      nh.param<std::string>("/robot_description_raisim", "");
  auto robot_description_pinocchio =
      nh.param<std::string>("/robot_description_pinocchio", "");
  auto object_description_raisim =
      nh.param<std::string>("/object_description_raisim", "");
  ROS_INFO_STREAM("Robot & Object Description Raisim Loaded");

  auto simulation = std::make_shared<OMAVVelocityDynamicsRos>(
      nh, robot_description_raisim, object_description_raisim, 0.015);
  ROS_INFO_STREAM("Simulation Created");
  // set initial state
  observation_t x = observation_t::Zero(simulation->get_state_dimension());
  auto x0 = nh.param<std::vector<double>>("initial_configuration", {});
  for (size_t i = 0; i < x0.size(); i++)
    x(i) = x0[i];
  // Set nominal state
  observation_t x_nom = observation_t::Zero(18);

  ROS_INFO_STREAM("Resetting initial state to " << x.transpose());
  simulation->reset(x);

  // init control input
  mppi::DynamicsBase::input_t u = input_t::Zero(6);

  bool static_optimization = nh.param<bool>("static_optimization", false);
  auto sim_dt = nh.param<double>("sim_dt", 0.015);
  double sim_time = 0.0;

  // init the controller
  bool ok = controller.init();
  if (!ok)
    throw std::runtime_error("Failed to initialize controller");

  if (running_rotors) {
    while (omav_trajectory_node->odometry_bool_) {
      ROS_INFO_STREAM_ONCE("No Odometry recieved yet");
      ros::spinOnce();
    }
  }
  ROS_INFO_STREAM("First Odometry recieved");
  // Set fir odomety value as reference
  if (running_rotors) {
    omav_trajectory_node->get_odometry(x);
    controller.set_reference(x);
  }

  // start controller
  bool sequential;
  nh.param<bool>("sequential", sequential, false);
  if (!sequential)
    controller.start();
  // do some timing
  double elapsed;
  std::chrono::time_point<std::chrono::steady_clock> start, end;

  while (ros::ok()) {
    start = std::chrono::steady_clock::now();
    if (omav_trajectory_node->rqt_cost_bool_) {
      omav_trajectory_node->rqt_cost_bool_ =
          controller.update_cost_param(omav_trajectory_node->rqt_cost_);
      ROS_INFO_STREAM("New Cost Param Set");
    }
    if (omav_trajectory_node->reset_object_) {
      x(13) = 0;
      simulation->reset(x);
      omav_trajectory_node->reset_object_ = false;
      ROS_INFO_STREAM("Reset Object");
    }

    if (sequential) {
      controller.update_reference();
      controller.set_observation(x, sim_time);
      controller.update_policy();
      controller.get_input_state(x, x_nom, u, sim_time);
      controller.publish_ros_default();
      // Additional publisher for additional visualization
      controller.publish_optimal_rollout();
      controller.publish_trajectories();

    } else {
      if (running_rotors) {
        omav_trajectory_node->get_odometry(x);
      }
      controller.set_observation(x, sim_time);
      controller.get_input_state(x, x_nom, u, sim_time);
      controller.publish_optimal_rollout();
      controller.publish_trajectories();
    }
    if (!running_rotors) {
      x = simulation->step(u, sim_dt);
      simulation->publish_ros();
      sim_time += sim_dt;
    }

    end = std::chrono::steady_clock::now();
    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
                  .count() /
              1000.0;
    if (sim_dt - elapsed > 0)
      ros::Duration(sim_dt - elapsed).sleep();
    else
      ROS_INFO_STREAM_THROTTLE(
          3.0, "Slower than real-time: " << elapsed / sim_dt << "x slower.");

    ros::spinOnce();
  }
}
