/*!
 * @file     controller_interface.cpp
 * @author   Matthias Studiger
 * @date     10.04.2021
 * @version  1.0
 * @brief    description
 */

#include <mppi_omav_interaction/controller_interface.h>

using namespace omav_interaction;

bool OMAVControllerInterface::init_ros() {
  detailed_publishing_ = nh_.param<bool>("detailed_publishing", false);
  publish_all_trajectories_ =
      nh_.param<bool>("publish_all_trajectories", false);
  detailed_publishing_rate_ = nh_.param<double>("detailed_publishing_rate", 10);

  // Initialize publisher
  cmd_multi_dof_joint_trajectory_pub_ =
      nh_public_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);

  if (publish_all_trajectories_) {
    trajectory_publisher_ =
        nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
            "debug/all_rollouts", 1);
  }

  if (detailed_publishing_) {
    ROS_INFO("[mppi_omav_interaction] Detailed publishing enabled.");

    optimal_rollout_publisher_ =
        nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
            "debug/optimal_rollout", 1);

    // Publish current pose reference used by mppi
    mppi_reference_publisher_ =
        nh_.advertise<geometry_msgs::PoseStamped>("debug/mppi_reference", 1);

    // Publish current object state used by mppi
    object_state_publisher_ =
        nh_.advertise<sensor_msgs::JointState>("debug/object/joint_state", 1);

    cost_publisher_ =
        nh_.advertise<mppi_ros::Array>("debug/cost_components", 1);
    normalized_force_publisher_ =
        nh_.advertise<visualization_msgs::MarkerArray>(
            "debug/normalized_forces", 1);
    pub_marker_hook_ =
        nh_.advertise<visualization_msgs::Marker>("debug/hook_pos", 1);
  } else {
    ROS_WARN("[mppi_omav_interaction] Detailed publishing disabled.");
  }

  // Initialize subscriber
  reference_subscriber_ =
      nh_.subscribe("/mppi_pose_desired", 1,
                    &OMAVControllerInterface::desired_pose_callback, this);
  mode_subscriber_ = nh_.subscribe(
      "/mppi_omav_mode", 1, &OMAVControllerInterface::mode_callback, this);
  object_reference_subscriber_ =
      nh_.subscribe("/mppi_object_desired", 1,
                    &OMAVControllerInterface::object_reference_callback, this);

  object_state_.name = {"articulation_joint"};
  object_state_.position.resize(1);
  return true;
}

void OMAVControllerInterface::setTask(const std::string &str) {
  if (str.compare("shelf") == 0) {
    ROS_INFO("[mppi_omav_interaction] Setting shelf as task.");
    task_ = InteractionTask::Shelf;
  } else if (str.compare("valve") == 0) {
    ROS_INFO("[mppi_omav_interaction] Setting valve as task.");
    task_ = InteractionTask::Valve;
  } else {
    ROS_ERROR(
        "[mppi_omav_interaction] Could not identify interaction task. Shutting "
        "down.");
    ros::shutdown();
  }
}

bool OMAVControllerInterface::set_controller(
    std::shared_ptr<mppi::PathIntegral> &controller) {
  // -------------------------------
  // config
  // -------------------------------
  std::string config_file;
  if (!nh_.param<std::string>("/config_file", config_file, "")) {
    throw std::runtime_error(
        "Could not parse config_file. Is the parameter set?");
  }
  if (!config_.init_from_file(config_file)) {
    ROS_ERROR_STREAM(
        "[mppi_omav_interaction] Failed to init solver options from "
        << config_file);
    return false;
  }
  if (!nh_.param<std::string>("/robot_description_raisim",
                              robot_description_raisim_, "")) {
    throw std::runtime_error(
        "Could not parse robot_description_raisim. Is the parameter set?");
  }
  if (!nh_.param<std::string>("/object_description", object_description_, "")) {
    throw std::runtime_error(
        "Could not parse object_description. Is the parameter set?");
  }
  if (!nh_.param<std::string>("/robot_description_pinocchio",
                              robot_description_pinocchio_, "")) {
    throw std::runtime_error(
        "Could not parse robot_description_pinocchio. Is the parameter set?");
  }

  // -------------------------------
  // dynamics
  // -------------------------------
  OMAVVelocityDynamics::omav_dynamics_ptr dynamics =
      std::make_shared<OMAVVelocityDynamics>(
          robot_description_raisim_, object_description_, config_.step_size);

  // -------------------------------
  // cost
  // -------------------------------
  if (task_ == InteractionTask::Shelf) {
    ROS_INFO("[mppi_omav_interaction] Using shelf as task.");
    if (!cost_param_shelf_.parse_from_ros(nh_)) {
      ROS_ERROR("[mppi_omav_interaction] Failed to parse cost parameters.");
      return false;
    }
    ROS_INFO_STREAM("Successfully parsed cost params: \n" << cost_param_shelf_);
    cost_shelf_ = std::make_shared<OMAVInteractionCostShelf>(
        robot_description_pinocchio_, object_description_, &cost_param_shelf_);
    controller =
        std::make_shared<mppi::PathIntegral>(dynamics, cost_shelf_, config_);
  } else if (task_ == InteractionTask::Valve) {
    ROS_INFO("[mppi_omav_interaction] Using valve as task.");
    if (!cost_param_valve_.parse_from_ros(nh_)) {
      ROS_ERROR("[mppi_omav_interaction] Failed to parse cost parameters.");
      return false;
    }
    ROS_INFO_STREAM("Successfully parsed cost params: \n" << cost_param_valve_);
    cost_valve_ = std::make_shared<OMAVInteractionCostValve>(
        robot_description_pinocchio_, object_description_, &cost_param_valve_);
    controller =
        std::make_shared<mppi::PathIntegral>(dynamics, cost_valve_, config_);
  } else {
    ROS_ERROR("[mppi_omav_interaction] No task selected. Shutting down.");
    ros::shutdown();
  }

  // -------------------------------
  // initialize reference
  // -------------------------------
  double x_goal_position, y_goal_position, z_goal_position, w_goal_quaternion,
      x_goal_quaternion, y_goal_quaternion, z_goal_quaternion;
  nh_.param<double>("goal_position_x", x_goal_position, 0.0);
  nh_.param<double>("goal_position_y", y_goal_position, 0.0);
  nh_.param<double>("goal_position_z", z_goal_position, 0.0);
  nh_.param<double>("goal_quaternion_w", w_goal_quaternion, 1.0);
  nh_.param<double>("goal_quaternion_x", x_goal_quaternion, 0.0);
  nh_.param<double>("goal_quaternion_y", y_goal_quaternion, 0.0);
  nh_.param<double>("goal_quaternion_z", z_goal_quaternion, 0.0);

  ref_.rr.resize(
      1, mppi::observation_t::Zero(reference_description::SIZE_REFERENCE));
  ref_.rr[0](reference_description::MAV_GOAL_POSITION_X_WORLD) =
      x_goal_position;
  ref_.rr[0](reference_description::MAV_GOAL_POSITION_Y_WORLD) =
      y_goal_position;
  ref_.rr[0](reference_description::MAV_GOAL_POSITION_Z_WORLD) =
      z_goal_position;
  ref_.rr[0](reference_description::MAV_GOAL_ORIENTATION_W_WORLD) =
      w_goal_quaternion;
  ref_.rr[0](reference_description::MAV_GOAL_ORIENTATION_X_WORLD) =
      x_goal_quaternion;
  ref_.rr[0](reference_description::MAV_GOAL_ORIENTATION_Y_WORLD) =
      y_goal_quaternion;
  ref_.rr[0](reference_description::MAV_GOAL_ORIENTATION_Z_WORLD) =
      z_goal_quaternion;
  ref_.rr[0](reference_description::OBJECT_GOAL_ORIENTATION) = 0.0;
  ref_.rr[0](reference_description::INTERACTION_MODE) =
      interaction_mode::FREE_FLIGHT;

  ROS_INFO_STREAM("Reference initialized with: " << ref_.rr[0].transpose());
  return true;
}

void OMAVControllerInterface::getDynamicsPtr(
    std::vector<std::shared_ptr<OMAVVelocityDynamics>> &omav_dynamics_v) {
  // Get pointer to base class dynamics:
  std::vector<mppi::DynamicsBase::dynamics_ptr> *dynamics_v;
  mppi::DynamicsBase::dynamics_ptr *dynamics;
  get_controller()->get_dynamics(&dynamics_v, &dynamics);
  size_t n = dynamics_v->size();
  omav_dynamics_v.clear();
  omav_dynamics_v.resize(n + 1);

  for (size_t i = 0; i < n; i++) {
    std::shared_ptr<OMAVVelocityDynamics> omav_dynamics =
        std::dynamic_pointer_cast<OMAVVelocityDynamics>(dynamics_v->at(i));
    omav_dynamics_v[i] = omav_dynamics;
  }
  omav_dynamics_v[n] =
      std::dynamic_pointer_cast<OMAVVelocityDynamics>(*dynamics);
}

void OMAVControllerInterface::desired_pose_callback(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  // std::unique_lock<std::mutex> lock(reference_mutex_);
  ref_.rr.resize(1);
  ref_.tt.resize(1);
  ref_.rr[0](reference_description::MAV_GOAL_POSITION_X_WORLD) =
      msg->pose.position.x;
  ref_.rr[0](reference_description::MAV_GOAL_POSITION_Y_WORLD) =
      msg->pose.position.y;
  ref_.rr[0](reference_description::MAV_GOAL_POSITION_Z_WORLD) =
      msg->pose.position.z;
  ref_.rr[0](reference_description::MAV_GOAL_ORIENTATION_W_WORLD) =
      msg->pose.orientation.w;
  ref_.rr[0](reference_description::MAV_GOAL_ORIENTATION_X_WORLD) =
      msg->pose.orientation.x;
  ref_.rr[0](reference_description::MAV_GOAL_ORIENTATION_Y_WORLD) =
      msg->pose.orientation.y;
  ref_.rr[0](reference_description::MAV_GOAL_ORIENTATION_Z_WORLD) =
      msg->pose.orientation.z;
  ref_.tt[0] = msg->header.stamp.toSec();
  std::cout << "Desired Pose Callback..." << std::endl;
  get_controller()->set_reference_trajectory(ref_);
}

void OMAVControllerInterface::updateValveReference(const double &ref_angle) {
  ref_.rr.resize(1);
  ref_.tt.resize(1);
  ref_.rr[0](reference_description::OBJECT_GOAL_ORIENTATION) = ref_angle;
  ref_.tt[0] = ros::Time::now().toSec();
  get_controller()->set_reference_trajectory(ref_);
}

void OMAVControllerInterface::updateValveReferenceDynamic(
    const double &start_angle, const double &end_angle, const double &t_start) {
  mppi::observation_t ref0 = ref_.rr[0];
  const size_t kN = 50;
  const double kHorizon_time = 1.0;
  ref_.rr.resize(kN);
  ref_.tt.resize(kN);
  for (size_t i = 0; i < kN; i++) {
    ref_.tt[i] = t_start + static_cast<double>(i) / kN * kHorizon_time;
    ref_.rr[i] = ref0;
    ref_.rr[i](reference_description::OBJECT_GOAL_ORIENTATION) =
        start_angle + static_cast<double>(i) / kN * (end_angle - start_angle);
  }
  get_controller()->set_reference_trajectory(ref_);
}

void OMAVControllerInterface::mode_callback(
    const std_msgs::Int64ConstPtr &msg) {
  // std::unique_lock<std::mutex> lock(reference_mutex_);
  setInteractionMode(msg->data);
}

void OMAVControllerInterface::setInteractionMode(const int &mode) {
  for (size_t i = 0; i < ref_.rr.size(); i++) {
    ref_.rr[i](reference_description::INTERACTION_MODE) = mode;
  }
  ROS_INFO_STREAM("Switching to mode: " << mode);
  get_controller()->set_reference_trajectory(ref_);
}

void OMAVControllerInterface::object_reference_callback(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  // std::unique_lock<std::mutex> lock(reference_mutex_);
  for (size_t i = 0; i < ref_.rr.size(); i++) {
    ref_.rr[i](reference_description::OBJECT_GOAL_ORIENTATION) =
        msg->pose.position.x;
  }
  get_controller()->set_reference_trajectory(ref_);
}

bool OMAVControllerInterface::update_reference() {
  if (!reference_set_) get_controller()->set_reference_trajectory(ref_);
  reference_set_ = true;
  return true;
}

void OMAVControllerInterface::getCostParamShelf(
    OMAVInteractionCostShelfParam &cost_param) const {
  cost_param = cost_param_shelf_;
}

void OMAVControllerInterface::getCostParamValve(
    OMAVInteractionCostValveParam &cost_param) const {
  cost_param = cost_param_valve_;
}

bool OMAVControllerInterface::update_cost_param_shelf(
    const OMAVInteractionCostShelfParam &cost_param) {
  cost_param_shelf_ = cost_param;
  return false;
}

bool OMAVControllerInterface::update_cost_param_valve(
    const OMAVInteractionCostValveParam &cost_param) {
  cost_param_valve_ = cost_param;
  return false;
}

bool OMAVControllerInterface::set_initial_reference(const observation_t &x) {
  ref_.tt.resize(1, 0.0);
  ref_.rr.resize(1, observation_t::Zero(reference_description::SIZE_REFERENCE));
  ref_.rr[0](reference_description::MAV_GOAL_POSITION_X_WORLD) =
      x(omav_state_description::MAV_POSITION_X_WORLD);
  ref_.rr[0](reference_description::MAV_GOAL_POSITION_Y_WORLD) =
      x(omav_state_description::MAV_POSITION_Y_WORLD);
  ref_.rr[0](reference_description::MAV_GOAL_POSITION_Z_WORLD) =
      x(omav_state_description::MAV_POSITION_Z_WORLD);
  ref_.rr[0](reference_description::MAV_GOAL_ORIENTATION_W_WORLD) =
      x(omav_state_description::MAV_ORIENTATION_W_WORLD);
  ref_.rr[0](reference_description::MAV_GOAL_ORIENTATION_X_WORLD) =
      x(omav_state_description::MAV_ORIENTATION_X_WORLD);
  ref_.rr[0](reference_description::MAV_GOAL_ORIENTATION_Y_WORLD) =
      x(omav_state_description::MAV_ORIENTATION_Y_WORLD);
  ref_.rr[0](reference_description::MAV_GOAL_ORIENTATION_Z_WORLD) =
      x(omav_state_description::MAV_ORIENTATION_Z_WORLD);
  get_controller()->set_reference_trajectory(ref_);
  return true;
}

void OMAVControllerInterface::publish_ros() {
  const ros::Time t_now = ros::Time::now();
  static ros::Time t_last_published = ros::Time::now();

  if (!get_controller()->get_optimal_rollout_for_trajectory(
          xx_opt_, uu_opt_, tt_opt_, t_now.toSec())) {
    ROS_ERROR("Error getting optimal rollout.");
    return;
  }

  publish_command_trajectory(xx_opt_, uu_opt_, tt_opt_);

  if ((t_now - t_last_published).toSec() >= 1. / detailed_publishing_rate_) {
    if (detailed_publishing_) {
      publish_optimal_rollout();

      // update object state visualization
      object_state_.header.stamp = t_now;
      object_state_.position[0] =
          xx_opt_[0](omav_state_description::OBJECT_ORIENTATION);
      object_state_publisher_.publish(object_state_);
    }

    if (publish_all_trajectories_) {
      publish_all_trajectories();
    }

    t_last_published = t_now;
  }
}

void OMAVControllerInterface::publish_command_trajectory(
    const mppi::observation_array_t &x_opt, const mppi::input_array_t &u_opt,
    const std::vector<double> &tt) {
  conversions::to_trajectory_msg(x_opt, u_opt, tt, damping_,
                                 current_trajectory_msg_);
  cmd_multi_dof_joint_trajectory_pub_.publish(current_trajectory_msg_);
  published_trajectory_ = true;
}

bool OMAVControllerInterface::get_current_trajectory(
    trajectory_msgs::MultiDOFJointTrajectory *current_trajectory_msg) const {
  if (published_trajectory_ && current_trajectory_msg_.points.size() > 0) {
    current_trajectory_msg->points.reserve(
        current_trajectory_msg_.points.size());
    *current_trajectory_msg = current_trajectory_msg_;
    return true;
  }
  return false;
}

/**
 * @brief      Publish all rollouts
 */
void OMAVControllerInterface::publish_all_trajectories() {
  std::vector<mppi::Rollout> rollouts;
  if (detailed_publishing_ &&
      get_controller()->get_rollout_trajectories(rollouts)) {
    // geometry_msgs::PoseArray trajectory_array;
    trajectory_msgs::MultiDOFJointTrajectory trajectory_array;
    geometry_msgs::Pose current_trajectory_pose;
    trajectory_array.header.frame_id = "world";
    trajectory_array.header.stamp = ros::Time(rollouts.front().tt[0]);
    std::vector<Eigen::VectorXd> xx_current_trajectory;
    // Iterate through rollouts:
    for (const auto &rollout : rollouts) {
      xx_current_trajectory = rollout.xx;
      for (const auto &xx_current_trajectory_point : xx_current_trajectory) {
        trajectory_msgs::MultiDOFJointTrajectoryPoint current_trajectory_point;
        conversions::MultiDofJointTrajectoryPointFromState(
            xx_current_trajectory_point, current_trajectory_point);
        trajectory_array.points.push_back(current_trajectory_point);
      }
    }
    trajectory_publisher_.publish(trajectory_array);
  }
}

void OMAVControllerInterface::publish_optimal_rollout() {
  std_msgs::Header header;
  header.frame_id = "world";
  header.stamp = ros::Time(tt_opt_.front());

  // Publish single pose reference:
  geometry_msgs::PoseStamped mppi_reference;
  mppi_reference.header.frame_id = "world";
  mppi_reference.pose.position.x =
      ref_.rr[0](reference_description::MAV_GOAL_POSITION_X_WORLD);
  mppi_reference.pose.position.y =
      ref_.rr[0](reference_description::MAV_GOAL_POSITION_Y_WORLD);
  mppi_reference.pose.position.z =
      ref_.rr[0](reference_description::MAV_GOAL_POSITION_Z_WORLD);
  mppi_reference.pose.orientation.w =
      ref_.rr[0](reference_description::MAV_GOAL_ORIENTATION_W_WORLD);
  mppi_reference.pose.orientation.x =
      ref_.rr[0](reference_description::MAV_GOAL_ORIENTATION_X_WORLD);
  mppi_reference.pose.orientation.y =
      ref_.rr[0](reference_description::MAV_GOAL_ORIENTATION_Y_WORLD);
  mppi_reference.pose.orientation.z =
      ref_.rr[0](reference_description::MAV_GOAL_ORIENTATION_Z_WORLD);
  mppi_reference_publisher_.publish(mppi_reference);

  // Compute cost components and publish
  if (task_ == InteractionTask::Shelf) {
    publishCostInfo(cost_shelf_, header);
  } else if (task_ == InteractionTask::Valve) {
    publishCostInfo(cost_valve_, header);
  }

  // Publish optimal rollout (trajectory, object ref. information)
  trajectory_msgs::MultiDOFJointTrajectory msg;
  toMultiDofJointTrajectory(msg);
  optimal_rollout_publisher_.publish(msg);

  // Publish hook position for visualization
  publishHookPos(header);
}

void OMAVControllerInterface::toMultiDofJointTrajectory(
    trajectory_msgs::MultiDOFJointTrajectory &t) const {
  t.points.clear();
  ros::Time stamp = ros::Time(tt_opt_.front());
  t.header.stamp = stamp;
  t.header.frame_id = "world";
  geometry_msgs::Transform tf;
  geometry_msgs::Twist vel;
  geometry_msgs::Twist acc;

  // Structure:
  // tf[0] = position
  // tf[2] = object information

  for (size_t i = 0; i < xx_opt_.size(); i++) {
    tf::vectorEigenToMsg(xx_opt_[i].head<3>(), tf.translation);
    tf::quaternionEigenToMsg(Eigen::Quaterniond(xx_opt_[i].segment<4>(3)),
                             tf.rotation);
    tf::vectorEigenToMsg(
        xx_opt_[i].segment<3>(
            omav_state_description::MAV_LINEAR_VELOCITY_X_WORLD),
        vel.linear);
    tf::vectorEigenToMsg(
        xx_opt_[i].segment<3>(
            omav_state_description::MAV_ANGULAR_VELOCITY_X_BODY),
        vel.angular);
    tf::vectorEigenToMsg(
        uu_opt_[i].segment<3>(
            control_input_description::MAV_LINEAR_ACCELERATION_X_DESIRED_WORLD),
        acc.linear);
    tf::vectorEigenToMsg(
        uu_opt_[i].segment<3>(
            control_input_description::MAV_ANGULAR_ACCELERATION_X_DESIRED_BODY),
        acc.angular);
    trajectory_msgs::MultiDOFJointTrajectoryPoint point;
    point.transforms.push_back(tf);
    point.velocities.push_back(vel);
    point.accelerations.push_back(acc);
    tf::vectorEigenToMsg(
        xx_opt_[i].segment<3>(
            omav_state_description::MAV_LINEAR_VELOCITY_X_DESIRED_WORLD),
        vel.linear);
    tf::vectorEigenToMsg(
        xx_opt_[i].segment<3>(
            omav_state_description::MAV_ANGULAR_VELOCITY_X_DESIRED_BODY),
        vel.angular);
    point.velocities.push_back(vel);
    // Contact force:
    tf::vectorEigenToMsg(
        xx_opt_[i].segment<3>(omav_state_description::INTERACTION_FORCE_X),
        vel.linear);
    point.velocities.push_back(vel);
    tf = geometry_msgs::Transform();
    tf.translation.x = xx_opt_[i](omav_state_description::OBJECT_ORIENTATION);
    tf.translation.y = xx_opt_[i](omav_state_description::OBJECT_VELOCITY);
    tf.translation.z =
        ref_interpolated_[i](reference_description::OBJECT_GOAL_ORIENTATION);
    point.transforms.push_back(tf);
    t.points.push_back(point);
  }
}

void OMAVControllerInterface::publishHookPos(
    const std_msgs::Header &header) const {
  const size_t N = xx_opt_.size();
  if (hook_pos_.size() == N) {
    visualization_msgs::Marker marker_hook;
    marker_hook.header = header;
    marker_hook.header.frame_id = "world";
    marker_hook.type = visualization_msgs::Marker::POINTS;
    marker_hook.pose.orientation.x = 0;
    marker_hook.pose.orientation.y = 0;
    marker_hook.pose.orientation.z = 0;
    marker_hook.pose.orientation.w = 1;
    marker_hook.pose.position.x = 0;
    marker_hook.pose.position.y = 0;
    marker_hook.pose.position.z = 0;
    marker_hook.scale.x = 0.02;
    marker_hook.scale.y = 0.02;
    marker_hook.scale.z = 0.02;
    marker_hook.color.a = 1.0;
    marker_hook.color.r = 0.0;
    marker_hook.color.g = 1.0;
    marker_hook.color.b = 0.0;
    geometry_msgs::Point point;
    std_msgs::ColorRGBA color;
    color.a = 1.0;
    color.b = 0.0;
    for (size_t i = 0; i < N; i++) {
      point.x = hook_pos_[i](0);
      point.y = hook_pos_[i](1);
      point.z = hook_pos_[i](2);
      color.r = static_cast<float>(i) / N;
      color.g = 1.0f - static_cast<float>(i) / N;

      marker_hook.points.push_back(point);
      marker_hook.colors.push_back(color);
    }
    pub_marker_hook_.publish(marker_hook);
  }
}

template <class T>
void OMAVControllerInterface::publishCostInfo(const T &cost,
                                              const std_msgs::Header &header) {
  // Info about costs:
  Eigen::Matrix<double, cost_description::SIZE_COST_VECTOR, 1> cost_vector;
  cost_vector.setZero();

  visualization_msgs::MarkerArray force_marker_array;

  visualization_msgs::Marker force_marker;
  conversions::arrow_initialization(force_marker);
  force_marker.header = header;

  hook_pos_.resize(xx_opt_.size());
  ref_interpolated_.resize(xx_opt_.size());

  for (size_t i = 0; i < xx_opt_.size(); i++) {
    // Cost publishing
    cost->get_stage_cost(xx_opt_[i], tt_opt_[i]);
    ref_interpolated_[i] = cost->r_;
    cost_vector += cost->cost_vector_;

    Eigen::Vector3d force_normed =
        xx_opt_[i].segment<3>(omav_state_description::INTERACTION_FORCE_X);

    force_marker.points[0].x = cost->hook_pos_(0);
    force_marker.points[0].y = cost->hook_pos_(1);
    force_marker.points[0].z = cost->hook_pos_(2);
    force_marker.points[1].x = cost->hook_pos_(0) + force_normed(0);
    force_marker.points[1].y = cost->hook_pos_(1) + force_normed(1);
    force_marker.points[1].z = cost->hook_pos_(2) + force_normed(2);
    force_marker.id = i;

    force_marker_array.markers.push_back(force_marker);
    hook_pos_[i] = cost->hook_pos_;
  }
  normalized_force_publisher_.publish(force_marker_array);

  mppi_ros::Array cost_array_message;
  cost_array_message.header = header;
  for (size_t i = 0; i < cost_description::SIZE_COST_VECTOR; i++) {
    cost_array_message.array.push_back(static_cast<float>(cost_vector(i)));
  }
  cost_array_message.array.push_back(cost_vector.sum());
  cost_array_message.array.push_back(
      ref_.rr[0](reference_description::INTERACTION_MODE));

  cost_publisher_.publish(cost_array_message);
}

// namespace omav_interaction
