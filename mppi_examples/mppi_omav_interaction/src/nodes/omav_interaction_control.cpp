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
      reference_param_server_(
          ros::NodeHandle(private_nh, "reference_parameters")),
      cost_param_server_(ros::NodeHandle(private_nh, "cost_parameters")) {
  initializePublishers();
  initializeSubscribers();
  object_state_ << 0, 0;
  // setup dynamic reconfigure
  dynamic_reconfigure::Server<
      mppi_omav_interaction::MPPIOmavReferenceConfig>::CallbackType f;
  dynamic_reconfigure::Server<
      mppi_omav_interaction::MPPIOmavCostConfig>::CallbackType g;
  f = boost::bind(&OmavTrajectoryGenerator::ReferenceParamCallback, this, _1,
                  _2);
  g = boost::bind(&OmavTrajectoryGenerator::CostParamCallback, this, _1, _2);
  reference_param_server_.setCallback(f);
  cost_param_server_.setCallback(g);
}

OmavTrajectoryGenerator::~OmavTrajectoryGenerator() {}

void OmavTrajectoryGenerator::initializeSubscribers() {
  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                &OmavTrajectoryGenerator::odometryCallback,
                                this, ros::TransportHints().tcpNoDelay());
  object_state_sub_ = nh_.subscribe("/observer/object/joint_state", 10,
                                    &OmavTrajectoryGenerator::objectCallback,
                                    this, ros::TransportHints().tcpNoDelay());
  ROS_INFO_STREAM("Subscribers initialized");
  odometry_bool_ = true;
}

void OmavTrajectoryGenerator::initializePublishers() {
  reference_publisher_ =
      nh_.advertise<geometry_msgs::PoseStamped>("/mppi_pose_desired", 10);
}

void OmavTrajectoryGenerator::odometryCallback(
    const nav_msgs::OdometryConstPtr &odometry_msg) {
  mav_msgs::eigenOdometryFromMsg(*odometry_msg, &current_odometry_);
  ROS_INFO_ONCE("MPPI got first odometry message");
  odometry_bool_ = false;
}

void OmavTrajectoryGenerator::objectCallback(
    const sensor_msgs::JointState &object_msg) {
  object_state_(0) = object_msg.position[0];
  object_state_(1) = object_msg.velocity[0];
  ROS_INFO_ONCE("MPPI got first object state message");
}

void OmavTrajectoryGenerator::get_odometry(observation_t &x) {
  x.head<3>() = current_odometry_.position_W;
  x(3) = current_odometry_.orientation_W_B.w();
  x.segment<3>(4) = current_odometry_.orientation_W_B.vec();
  x.segment<3>(7) = current_odometry_.getVelocityWorld();
  x.segment<3>(10) = current_odometry_.angular_velocity_B;
  x.segment<2>(13) = object_state_;
}

void OmavTrajectoryGenerator::ReferenceParamCallback(
    mppi_omav_interaction::MPPIOmavReferenceConfig &config, uint32_t level) {
  if (config.reset) {
    config.reset = false;
    config.ref_pos_x = current_odometry_.position_W.x();
    config.ref_pos_y = current_odometry_.position_W.y();
    config.ref_pos_z = current_odometry_.position_W.z();
    Eigen::Vector3d euler_angles;
    current_odometry_.getEulerAngles(&euler_angles);
    config.ref_roll = euler_angles(0) * 360 / (2 * M_PI);
    config.ref_pitch = euler_angles(1) * 360 / (2 * M_PI);
    config.ref_yaw = euler_angles(2) * 360 / (2 * M_PI);
  } else {
    geometry_msgs::PoseStamped rqt_pose_msg;
    Eigen::VectorXd rqt_pose(7);
    Eigen::Quaterniond q;
    double euler_x = config.ref_roll * 2 * M_PI / 360;
    double euler_y = config.ref_pitch * 2 * M_PI / 360;
    double euler_z = config.ref_yaw * 2 * M_PI / 360;
    Eigen::AngleAxisd rollAngle(euler_x, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(euler_y, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(euler_z, Eigen::Vector3d::UnitZ());
    q = rollAngle * pitchAngle * yawAngle;
    rqt_pose << config.ref_pos_x, config.ref_pos_y, config.ref_pos_z, q.w(),
        q.x(), q.y(), q.z();
    omav_interaction::conversions::PoseStampedMsgFromVector(rqt_pose,
                                                            rqt_pose_msg);
    reference_publisher_.publish(rqt_pose_msg);

    if (config.reset_object) {
      reset_object_ = true;
      config.reset_object = false;
    }
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
  rqt_cost_.Q_object = config.Q_object;
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
  rqt_cost_.contact_bool = config.contact_prohibitor;
  rqt_cost_bool_ = true;
}

int main(int argc, char **argv) {
  // Initialize Ros Node
  ros::init(argc, argv, "omav_control_node");
  ros::NodeHandle nh("~"), nh_public;
  // Check if running with omav interface
  bool running_rotors = nh.param<bool>("running_rotors", false);
  ROS_INFO_STREAM(running_rotors);
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
  auto object_description = nh.param<std::string>("/object_description", "");
  ROS_INFO_STREAM("Robot & Object Description Raisim Loaded");

  auto simulation = std::make_shared<OMAVVelocityDynamicsRos>(
      nh, robot_description_raisim, object_description, 0.015);
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
    controller.set_initial_reference(x);
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
    ros::Rate r(200);
    if (sequential) {
      controller.update_reference();
      controller.set_observation(x, sim_time);
      controller.update_policy();
      controller.get_input_state(x, x_nom, u, sim_time);
      controller.publish_ros_default();
      // Additional publisher for additional visualization
      controller.publish_optimal_rollout();
      controller.publish_all_trajectories();

    } else {
      if (running_rotors) {
        omav_trajectory_node->get_odometry(x);
      }
      controller.set_observation(x, sim_time);
      controller.get_input_state(x, x_nom, u, sim_time);
      r.sleep();
      sim_time += 1 / 200;
    }
    if (!running_rotors) {
      x = simulation->step(u, sim_dt);
      simulation->publish_ros();
      sim_time += sim_dt;

      end = std::chrono::steady_clock::now();
      elapsed =
          std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
              .count() /
          1000.0;
      if (sim_dt - elapsed > 0)
        ros::Duration(sim_dt - elapsed).sleep();
      else
        ROS_INFO_STREAM_THROTTLE(
            3.0, "Slower than real-time: " << elapsed / sim_dt << "x slower.");
    }
    ros::spinOnce();
  }
}
