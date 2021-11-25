#include "mppi_omav_interaction/omav_trajectory_generator.h"

using namespace omav_interaction;

OmavTrajectoryGenerator::OmavTrajectoryGenerator(
    const ros::NodeHandle &nh, const ros::NodeHandle &private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      reference_param_server_(
          ros::NodeHandle(private_nh, "reference_parameters")),
      cost_param_server_(ros::NodeHandle(private_nh, "cost_parameters")) {
  initializePublishers();
  initializeSubscribers();
  // Initialize data to prevent problems
  object_state_.setZero();
  target_state_.orientation_W_B.w() = 1.0;
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
  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 10,
                                &OmavTrajectoryGenerator::odometryCallback,
                                this, ros::TransportHints().tcpNoDelay());
  position_target_sub_ = nh_.subscribe(
      "command/trajectory", 10, &OmavTrajectoryGenerator::TargetCallback, this,
      ros::TransportHints().tcpNoDelay());
  object_state_sub_ = nh_.subscribe("/shelf/joint_states", 10,
                                    &OmavTrajectoryGenerator::objectCallback,
                                    this, ros::TransportHints().tcpNoDelay());
  int i = 0;
  while (object_state_sub_.getNumPublishers() <= 0 && i < 10) {
    ros::Duration(1.0).sleep();
    ROS_WARN("[mppi_omav_interaction] Waiting for publisher of shelf state.");
    i++;
  }
  if (i == 10) {
    ROS_ERROR("[mppi_omav_interaction] Did not get shelf state publisher.");
    ros::shutdown();
  }
  ROS_INFO("[mppi_omav_interaction] Subscribers initialized");
  odometry_bool_ = true;
}

void OmavTrajectoryGenerator::initializePublishers() {
  reference_publisher_ =
      nh_.advertise<geometry_msgs::PoseStamped>("/mppi_pose_desired", 10);
}

void OmavTrajectoryGenerator::odometryCallback(
    const nav_msgs::OdometryConstPtr &odometry_msg) {
  mav_msgs::eigenOdometryFromMsg(*odometry_msg, &current_odometry_);
  ROS_INFO_ONCE("[mppi_omav_interaction] MPPI got odometry message");
  odometry_bool_ = false;
}

void OmavTrajectoryGenerator::objectCallback(
    const sensor_msgs::JointState &object_msg) {
  object_state_(0) = object_msg.position[0];
  object_state_(1) = object_msg.velocity[0];
  ROS_INFO_ONCE("[mppi_omav_interaction] MPPI got first object state message");
}

void OmavTrajectoryGenerator::TargetCallback(
    const trajectory_msgs::MultiDOFJointTrajectory &position_target_msg) {
  target_state_time_ = 0.0;
  first_trajectory_sent_ = true;
  set_target(position_target_msg.points[0]);
  current_trajectory_ = position_target_msg;
  shift_lock_ = false;
}

void OmavTrajectoryGenerator::get_odometry(observation_t &x) const {
  x.head<3>() = current_odometry_.position_W;
  x(3) = current_odometry_.orientation_W_B.w();
  x.segment<3>(4) = current_odometry_.orientation_W_B.vec();
  x.segment<3>(7) = current_odometry_.getVelocityWorld();
  x.segment<3>(10) = current_odometry_.angular_velocity_B;
  x.segment<2>(13) = object_state_;
  x.segment<3>(19) = target_state_.position_W;
  x(22) = target_state_.orientation_W_B.w();
  x.segment<3>(23) = target_state_.orientation_W_B.vec();
  x.segment<3>(26) = target_state_.velocity_W;
  x.segment<3>(29) = target_state_.angular_velocity_W;
  ROS_INFO_ONCE("[mppi_omav_interaction] MPPI got first state message");
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
  }
  geometry_msgs::PoseStamped rqt_pose_msg;
  Eigen::VectorXd rqt_pose(7);
  Eigen::Quaterniond q;
  omav_interaction::conversions::RPYtoQuaterniond(
      config.ref_roll, config.ref_pitch, config.ref_yaw, q);
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

void OmavTrajectoryGenerator::initialize_integrators(observation_t &x) {
  x.segment<3>(19) = current_odometry_.position_W;
  x(22) = current_odometry_.orientation_W_B.w();
  x.segment<3>(23) = current_odometry_.orientation_W_B.vec();
  x.segment<3>(26) = current_odometry_.getVelocityWorld();
  x.segment<3>(29) = current_odometry_.angular_velocity_B;

  target_state_.position_W = current_odometry_.position_W;
  target_state_.orientation_W_B.w() = current_odometry_.orientation_W_B.w();
  target_state_.orientation_W_B.vec() = current_odometry_.orientation_W_B.vec();
  target_state_.velocity_W = current_odometry_.getVelocityWorld();
  target_state_.angular_velocity_W = current_odometry_.angular_velocity_B;
}

void OmavTrajectoryGenerator::set_target(
    const trajectory_msgs::MultiDOFJointTrajectoryPoint &trajectory_msg_point) {
  Eigen::Vector3d position =
      mav_msgs::vector3FromMsg(trajectory_msg_point.transforms[0].translation);
  target_state_.position_W = position;
  target_state_.orientation_W_B.w() =
      trajectory_msg_point.transforms[0].rotation.w;
  target_state_.orientation_W_B.x() =
      trajectory_msg_point.transforms[0].rotation.x;
  target_state_.orientation_W_B.y() =
      trajectory_msg_point.transforms[0].rotation.y;
  target_state_.orientation_W_B.z() =
      trajectory_msg_point.transforms[0].rotation.z;
  Eigen::Vector3d velocity_lin =
      mav_msgs::vector3FromMsg(trajectory_msg_point.velocities[0].linear);
  target_state_.velocity_W = velocity_lin;
  Eigen::Vector3d velocity_ang =
      mav_msgs::vector3FromMsg(trajectory_msg_point.velocities[0].angular);
  target_state_.angular_velocity_W = velocity_ang;
}