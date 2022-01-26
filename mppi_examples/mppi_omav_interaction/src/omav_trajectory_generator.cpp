#include "mppi_omav_interaction/omav_trajectory_generator.h"

using namespace omav_interaction;

OmavTrajectoryGenerator::OmavTrajectoryGenerator(
    const ros::NodeHandle &nh, const ros::NodeHandle &private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      reference_param_server_(
          ros::NodeHandle(private_nh, "reference_parameters")),
      cost_shelf_param_server_(ros::NodeHandle(private_nh, "cost_shelf_parameters")),
      cost_valve_param_server_(
          ros::NodeHandle(private_nh, "cost_valve_parameters")) {
  // initializePublishers();
  initializeSubscribers();
  // Initialize data to prevent problems
  object_state_.setZero();
  target_state_.orientation_W_B.w() = 1.0;
  // setup dynamic reconfigure
  dynamic_reconfigure::Server<
      mppi_omav_interaction::MPPIOmavReferenceConfig>::CallbackType f;
  dynamic_reconfigure::Server<
      mppi_omav_interaction::MPPIOmavCostShelfConfig>::CallbackType g;
  dynamic_reconfigure::Server<
      mppi_omav_interaction::MPPIOmavCostValveConfig>::CallbackType h;
  f = boost::bind(&OmavTrajectoryGenerator::ReferenceParamCallback, this, _1,
                  _2);
  g = boost::bind(&OmavTrajectoryGenerator::CostShelfParamCallback, this, _1, _2);
  h = boost::bind(&OmavTrajectoryGenerator::CostValveParamCallback, this, _1,
                  _2);
  reference_param_server_.setCallback(f);
  cost_shelf_param_server_.setCallback(g);
  cost_valve_param_server_.setCallback(h);
}

OmavTrajectoryGenerator::~OmavTrajectoryGenerator() {}

void OmavTrajectoryGenerator::initializeSubscribers() {
  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                &OmavTrajectoryGenerator::odometryCallback,
                                this, ros::TransportHints().tcpNoDelay());
  position_target_sub_ = nh_.subscribe(
      "command/trajectory", 1, &OmavTrajectoryGenerator::TargetCallback, this,
      ros::TransportHints().tcpNoDelay());
  object_state_sub_ = nh_.subscribe("object_joint_states", 10,
                                    &OmavTrajectoryGenerator::objectCallback,
                                    this, ros::TransportHints().tcpNoDelay());
  int i = 0;
  while (object_state_sub_.getNumPublishers() <= 0 && i < 10) {
    ros::Duration(1.0).sleep();
    ROS_WARN("[mppi_omav_interaction] Waiting for publisher of object state.");
    i++;
  }
  if (i == 10) {
    ROS_ERROR("[mppi_omav_interaction] Did not get object state publisher.");
    ros::shutdown();
  }
  ROS_INFO("[mppi_omav_interaction] Subscribers initialized");
}

void OmavTrajectoryGenerator::initializePublishers() {
  reference_publisher_ =
      nh_.advertise<geometry_msgs::PoseStamped>("/mppi_pose_desired", 1);
}

void OmavTrajectoryGenerator::odometryCallback(
    const nav_msgs::OdometryConstPtr &odometry_msg) {
  mav_msgs::eigenOdometryFromMsg(*odometry_msg, &current_odometry_);
  is_new_odom_ = true;
  odometry_valid_ = true;
  ROS_INFO_ONCE("[mppi_omav_interaction] MPPI got odometry message");
}

void OmavTrajectoryGenerator::objectCallback(
    const sensor_msgs::JointState &object_msg) {
  object_state_(0) = object_msg.position[0];
  object_state_(1) = object_msg.velocity[0];
  object_valid_ = true;
  ROS_INFO_ONCE("[mppi_omav_interaction] MPPI got first object state message");
}

/**
 * @brief      Callback for a position target message
 *
 * @param[in]  position_target_msg  The position target message
 */
void OmavTrajectoryGenerator::TargetCallback(
    const trajectory_msgs::MultiDOFJointTrajectory &position_target_msg) {
  if (set_target(position_target_msg.points[0])) {
    last_target_received_ = position_target_msg.header.stamp;
    target_state_time_ = 0.0;
    first_trajectory_sent_ = true;
    current_trajectory_ = position_target_msg;
    shift_lock_ = false;
  }
}

bool OmavTrajectoryGenerator::get_state(observation_t &x) {
  if (!odometry_valid_ || !object_valid_) {
    return false;
  }
  getTargetStateFromTrajectory();

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
  return true;
}

bool OmavTrajectoryGenerator::getTargetStateFromTrajectory() {
  // Compute target state according to odometry timestamp:
  if (trajectory_available_ && current_trajectory_.points.size() > 0) {
    int64_t t_ref = current_trajectory_.header.stamp.toNSec();
    const int64_t t_odom_s = current_odometry_.timestamp_ns;
    size_t i = 0;
    while (t_ref < t_odom_s && i < current_trajectory_.points.size()) {
      t_ref = (current_trajectory_.header.stamp +
               current_trajectory_.points[i].time_from_start)
                  .toNSec();
      i++;
    }
    if (i == current_trajectory_.points.size()) {
      ROS_ERROR(
          "Target exceeded trajectory length, t_odom_s = %f, last traj t = %f",
          static_cast<double>(t_odom_s / 1e9),
          static_cast<double>(t_ref / 1e9));
      i = 0;
    }
    if (i > 1 && i < current_trajectory_.points.size()) {
      double t = static_cast<double>((t_odom_s - (t_ref - 15e6)) / 1e9) / 0.015;
      mav_msgs::EigenTrajectoryPoint target_state;
      omav_interaction::conversions::InterpolateTrajectoryPoints(
          current_trajectory_.points[i - 2], current_trajectory_.points[i - 1],
          t, &target_state);
      set_target(target_state);
    } else {
      set_target(current_trajectory_.points[i]);
      ROS_WARN("Was not able to interpolate trajectory points.");
    }
    return true;
  }
  ROS_ERROR("No trajectory available.");
  return false;
}

bool OmavTrajectoryGenerator::get_state(observation_t &x, double &timestamp,
                                        bool &is_new) {
  if (get_state(x)) {
    is_new = is_new_odom_;
    is_new_odom_ = false;
    timestamp = static_cast<double>(current_odometry_.timestamp_ns) / 1.e9;
  } else {
    return false;
  }
  return true;
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

void OmavTrajectoryGenerator::CostShelfParamCallback(
    mppi_omav_interaction::MPPIOmavCostShelfConfig &config, uint32_t level) {
  // Update OMAV pose cost
  rqt_cost_shelf_.Q_x_omav = config.Q_x_omav;
  rqt_cost_shelf_.Q_y_omav = config.Q_y_omav;
  rqt_cost_shelf_.Q_z_omav = config.Q_z_omav;
  rqt_cost_shelf_.Q_orientation = config.Q_orientation_omav;
  rqt_cost_shelf_.pose_costs << rqt_cost_shelf_.Q_x_omav,
      rqt_cost_shelf_.Q_y_omav, rqt_cost_shelf_.Q_z_omav,
      rqt_cost_shelf_.Q_orientation, rqt_cost_shelf_.Q_orientation,
      rqt_cost_shelf_.Q_orientation;
  rqt_cost_shelf_.Q_pose = rqt_cost_shelf_.pose_costs.asDiagonal();
  // Update Object cost
  rqt_cost_shelf_.Q_object = config.Q_object;
  // Update velocity cost
  rqt_cost_shelf_.Q_lin_vel = config.Q_linear_velocity;
  rqt_cost_shelf_.vel_costs << rqt_cost_shelf_.Q_lin_vel,
      rqt_cost_shelf_.Q_lin_vel, rqt_cost_shelf_.Q_lin_vel, config.Q_roll,
      config.Q_pitch, config.Q_yaw;
  rqt_cost_shelf_.Q_vel = rqt_cost_shelf_.vel_costs.asDiagonal();
  // Update Handle Hook cost components
  rqt_cost_shelf_.handle_hook_thresh = config.handle_hook_thresh;
  rqt_cost_shelf_.Q_handle_hook = config.handle_hook_cost;
  // Update floor cost components
  rqt_cost_shelf_.floor_thresh = config.floor_thresh;
  rqt_cost_shelf_.Q_floor = config.floor_cost;
  rqt_cost_shelf_.Q_efficiency = config.efficiency_cost;
  rqt_cost_shelf_.Q_force = config.force_cost;
  rqt_cost_shelf_.Q_torque = config.torque_cost;
  rqt_cost_shelf_.contact_bool = config.contact_prohibitor;
  rqt_cost_shelf_bool_ = true;
}

void OmavTrajectoryGenerator::CostValveParamCallback(
    mppi_omav_interaction::MPPIOmavCostValveConfig &config, uint32_t level) {
  // Update OMAV pose cost
  rqt_cost_valve_.Q_x_omav = config.Q_x_omav;
  rqt_cost_valve_.Q_y_omav = config.Q_y_omav;
  rqt_cost_valve_.Q_z_omav = config.Q_z_omav;
  rqt_cost_valve_.Q_orientation = config.Q_orientation_omav;
  rqt_cost_valve_.pose_costs << rqt_cost_valve_.Q_x_omav,
      rqt_cost_valve_.Q_y_omav, rqt_cost_valve_.Q_z_omav,
      rqt_cost_valve_.Q_orientation, rqt_cost_valve_.Q_orientation,
      rqt_cost_valve_.Q_orientation;
  rqt_cost_valve_.Q_pose = rqt_cost_valve_.pose_costs.asDiagonal();
  // Update Object cost
  rqt_cost_valve_.Q_object = config.Q_object;
  // Update velocity cost
  rqt_cost_valve_.Q_lin_vel = config.Q_linear_velocity;
  rqt_cost_valve_.vel_costs << rqt_cost_valve_.Q_lin_vel,
      rqt_cost_valve_.Q_lin_vel, rqt_cost_valve_.Q_lin_vel, config.Q_roll,
      config.Q_pitch, config.Q_yaw;
  rqt_cost_valve_.Q_vel = rqt_cost_valve_.vel_costs.asDiagonal();
  // Update Handle Hook cost components
  rqt_cost_valve_.handle_hook_thresh = config.handle_hook_thresh;
  rqt_cost_valve_.Q_handle_hook = config.handle_hook_cost;
  // Update floor cost components
  rqt_cost_valve_.floor_thresh = config.floor_thresh;
  rqt_cost_valve_.Q_floor = config.floor_cost;
  // rqt_cost_valve_.Q_efficiency = config.efficiency_cost;
  rqt_cost_valve_.Q_force = config.force_cost;
  // rqt_cost_valve_.Q_torque = config.torque_cost;
  rqt_cost_valve_.contact_bool = config.contact_prohibitor;
  rqt_cost_valve_bool_ = true;
}

bool OmavTrajectoryGenerator::initialize_integrators(observation_t &x) {
  if (!odometry_valid_) {
    return false;
  }
  x.segment<3>(19) = current_odometry_.position_W;
  x(22) = current_odometry_.orientation_W_B.w();
  x.segment<3>(23) = current_odometry_.orientation_W_B.vec();
  x.segment<3>(26) = current_odometry_.getVelocityWorld();
  x.segment<3>(29) = current_odometry_.angular_velocity_B;

  target_state_.position_W = current_odometry_.position_W;
  target_state_.orientation_W_B = current_odometry_.orientation_W_B;
  target_state_.velocity_W = current_odometry_.getVelocityWorld();
  target_state_.angular_velocity_W = current_odometry_.angular_velocity_B;
  return true;
}

/**
 * @brief      Sets a position target.
 *
 * @param[in]  trajectory_msg_point  The trajectory message point
 *
 * @return     true if successful, false otherwise.
 */
bool OmavTrajectoryGenerator::set_target(
    const trajectory_msgs::MultiDOFJointTrajectoryPoint &trajectory_msg_point) {
  if (trajectory_msg_point.transforms.size() <= 0) {
    ROS_ERROR("[mppi_omav_interaction] Target does not contain any reference.");
    return false;
  }
  target_state_.orientation_W_B =
      mav_msgs::quaternionFromMsg(trajectory_msg_point.transforms[0].rotation);
  target_state_.position_W =
      mav_msgs::vector3FromMsg(trajectory_msg_point.transforms[0].translation);
  target_state_.velocity_W =
      mav_msgs::vector3FromMsg(trajectory_msg_point.velocities[0].linear);
  target_state_.angular_velocity_W =
      mav_msgs::vector3FromMsg(trajectory_msg_point.velocities[0].angular);
  return true;
}

bool OmavTrajectoryGenerator::set_target(
    const mav_msgs::EigenTrajectoryPoint &target_state) {
  target_state_ = target_state;
  return true;
}

void OmavTrajectoryGenerator::setCurrentTrajectory(
    const trajectory_msgs::MultiDOFJointTrajectory &current_trajectory) {
  current_trajectory_ = current_trajectory;
  trajectory_available_ = true;
}