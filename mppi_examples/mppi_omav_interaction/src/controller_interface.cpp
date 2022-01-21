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

  // Initialize publisher
  optimal_rollout_publisher_ =
      nh_.advertise<geometry_msgs::PoseArray>("debug/optimal_rollout", 1);
  optimal_rollout_des_publisher_ = nh_.advertise<geometry_msgs::PoseArray>(
      "debug/optimal_rollout_desired", 1);
  optimal_linear_input_publisher_ =
      nh_.advertise<geometry_msgs::PoseArray>("debug/optimal_input_linear", 1);
  optimal_angular_input_publisher_ =
      nh_.advertise<geometry_msgs::PoseArray>("debug/optimal_input_angular", 1);
  optimal_rollout_ang_vel_ =
      nh_.advertise<geometry_msgs::PoseArray>("debug/optimal_ang_vel", 1);
  optimal_rollout_lin_vel_ =
      nh_.advertise<geometry_msgs::PoseArray>("debug/optimal_lin_vel", 1);
  cmd_multi_dof_joint_trajectory_pub_ =
      nh_public_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
  // Publish current pose reference used by mppi
  mppi_reference_publisher_ =
      nh_.advertise<geometry_msgs::Pose>("debug/mppi_reference", 1);
  // Publish current object state used by mppi
  object_state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("/object/joint_state", 1);

  if (detailed_publishing_) {
    ROS_INFO("[mppi_omav_interaction] Detailed publishing enabled.");
    trajectory_publisher_ =
        nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
            "debug/all_rollouts", 1);

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
    cost_shelf_ = std::make_shared<OMAVInteractionCost>(
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

  ref_.rr.resize(1, mppi::observation_t::Zero(9));
  ref_.rr[0](0) = x_goal_position;
  ref_.rr[0](1) = y_goal_position;
  ref_.rr[0](2) = z_goal_position;
  ref_.rr[0](3) = w_goal_quaternion;
  ref_.rr[0](4) = x_goal_quaternion;
  ref_.rr[0](5) = y_goal_quaternion;
  ref_.rr[0](6) = z_goal_quaternion;
  ref_.rr[0](7) = 0.0;
  // Initialize mode
  ref_.rr[0](8) = 0;
  ref_.tt.resize(1, 0.0);

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
  ref_.rr[0](0) = msg->pose.position.x;
  ref_.rr[0](1) = msg->pose.position.y;
  ref_.rr[0](2) = msg->pose.position.z;
  ref_.rr[0](3) = msg->pose.orientation.w;
  ref_.rr[0](4) = msg->pose.orientation.x;
  ref_.rr[0](5) = msg->pose.orientation.y;
  ref_.rr[0](6) = msg->pose.orientation.z;
  std::cout << "Desired Pose Callback..." << std::endl;
  get_controller()->set_reference_trajectory(ref_);
}

void OMAVControllerInterface::updateValveReference(const double &ref_angle) {
  ref_.rr[0](7) = ref_angle;
  get_controller()->set_reference_trajectory(ref_);
}

void OMAVControllerInterface::mode_callback(
    const std_msgs::Int64ConstPtr &msg) {
  // std::unique_lock<std::mutex> lock(reference_mutex_);
  ref_.rr[0](8) = msg->data;
  ROS_INFO_STREAM("Switching to mode:" << msg->data);
  get_controller()->set_reference_trajectory(ref_);
}

void OMAVControllerInterface::object_reference_callback(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  // std::unique_lock<std::mutex> lock(reference_mutex_);
  ref_.rr[0](7) = msg->pose.position.x;
  get_controller()->set_reference_trajectory(ref_);
}

bool OMAVControllerInterface::update_reference() {
  if (!reference_set_) get_controller()->set_reference_trajectory(ref_);
  reference_set_ = true;
  return true;
}

bool OMAVControllerInterface::update_cost_param_shelf(
    const OMAVInteractionCostParam &cost_param) {
  cost_param_shelf_ = cost_param;
  return false;
}

bool OMAVControllerInterface::update_cost_param_valve(
    const OMAVInteractionCostValveParam &cost_param) {
  cost_param_valve_ = cost_param;
  return false;
}

bool OMAVControllerInterface::set_initial_reference(const observation_t &x) {
  ref_.rr[0](0) = x(0);
  ref_.rr[0](1) = x(1);
  ref_.rr[0](2) = x(2);
  ref_.rr[0](3) = x(3);
  ref_.rr[0](4) = x(4);
  ref_.rr[0](5) = x(5);
  ref_.rr[0](6) = x(6);
  get_controller()->set_reference_trajectory(ref_);
  return true;
}

void OMAVControllerInterface::publish_ros() {
  const ros::Time t_now = ros::Time::now();
  if (!get_controller()->get_optimal_rollout_for_trajectory(
          xx_opt_, uu_opt_, tt_opt_, t_now.toSec())) {
    ROS_ERROR("Error getting optimal rollout.");
    return;
  }
  x0_ = get_controller()->get_current_observation();

  publish_trajectory(xx_opt_, uu_opt_, x0_, tt_opt_);
  publish_optimal_rollout();
  static int counter = 0;
  counter++;
  if (counter == 1) {
    publish_all_trajectories();
    counter = 0;
  }

  static tf::TransformBroadcaster odom_broadcaster;
  tf::Transform omav_odom;
  omav_odom.setOrigin(tf::Vector3(x0_(0), x0_(1), x0_(2)));
  omav_odom.setRotation(tf::Quaternion(x0_(4), x0_(5), x0_(6), x0_(3)));
  odom_broadcaster.sendTransform(
      tf::StampedTransform(omav_odom, t_now, "world", "odom_omav"));

  // update object state visualization
  object_state_.header.stamp = t_now;
  object_state_.position[0] = xx_opt_[0](13);
  object_state_publisher_.publish(object_state_);
}

/**
 * @brief      Publish optimal trajectory
 *
 * @param[in]  x_opt   Optimal states
 * @param[in]  u_opt   Optimal inputs
 */
void OMAVControllerInterface::publish_trajectory(
    const mppi::observation_array_t &x_opt, const mppi::input_array_t &u_opt,
    const mppi::observation_t &x0_opt, const std::vector<double> &tt) {
  // Shift trajectory for publishing since the OMAV controller just inserts the
  // trajectory while ignoring the actual message timestamp
  const double t_now = ros::Time::now().toSec();
  size_t offset = 0;
  while (offset < tt.size() - 1 && tt[offset] <= t_now) {
    offset++;
  }
  mppi::observation_array_t x_opt_offs =
      mppi::observation_array_t(x_opt.begin() + offset, x_opt.end());
  mppi::input_array_t u_opt_offs =
      mppi::input_array_t(u_opt.begin() + offset, u_opt.end());
  std::vector<double> tt_offs =
      std::vector<double>(tt.begin() + offset, tt.end());
  trajectory_msgs::MultiDOFJointTrajectory current_trajectory_msg_offs;
  conversions::to_trajectory_msg(x_opt, u_opt, tt, current_trajectory_msg_);
  conversions::to_trajectory_msg(x_opt_offs, u_opt_offs, tt_offs,
                                 current_trajectory_msg_offs);
  cmd_multi_dof_joint_trajectory_pub_.publish(current_trajectory_msg_offs);
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
  geometry_msgs::PoseArray optimal_rollout_array, optimal_rollout_array_des,
      optimal_inputs_lin_array, optimal_inputs_ang_array,
      optimal_rollout_lin_vel, optimal_rollout_ang_vel;
  geometry_msgs::Pose current_pose, current_pose_des, mppi_reference,
      current_input_lin, current_input_ang, current_lin_vel, current_ang_vel;

  std_msgs::Header header;
  header.frame_id = "world";
  header.stamp = ros::Time::now();
  optimal_rollout_array.header = header;
  optimal_rollout_array_des.header = header;
  optimal_inputs_lin_array.header = header;
  optimal_inputs_ang_array.header = header;
  optimal_rollout_lin_vel.header = header;
  optimal_rollout_ang_vel.header = header;

  conversions::PoseMsgForVelocityFromVector(x0_.segment<3>(7), current_lin_vel);
  conversions::PoseMsgForVelocityFromVector(x0_.segment<3>(10),
                                            current_ang_vel);
  optimal_rollout_lin_vel.poses.push_back(current_lin_vel);
  optimal_rollout_ang_vel.poses.push_back(current_ang_vel);

  for (size_t i = 0; i < xx_opt_.size(); i++) {
    conversions::PoseMsgFromVector(xx_opt_[i].head<7>(), current_pose);
    conversions::PoseMsgFromVector(xx_opt_[i].segment<7>(19), current_pose_des);
    conversions::PoseMsgForVelocityFromVector(uu_opt_[i].segment<3>(0),
                                              current_input_lin);
    conversions::PoseMsgForVelocityFromVector(uu_opt_[i].segment<3>(3),
                                              current_input_ang);
    conversions::PoseMsgForVelocityFromVector(xx_opt_[i].segment<3>(7),
                                              current_lin_vel);
    conversions::PoseMsgForVelocityFromVector(xx_opt_[i].segment<3>(10),
                                              current_ang_vel);
    optimal_rollout_array.poses.push_back(current_pose);
    optimal_rollout_array_des.poses.push_back(current_pose_des);
    optimal_inputs_lin_array.poses.push_back(current_input_lin);
    optimal_inputs_ang_array.poses.push_back(current_input_ang);
    optimal_rollout_lin_vel.poses.push_back(current_lin_vel);
    optimal_rollout_ang_vel.poses.push_back(current_ang_vel);
  }
  optimal_rollout_publisher_.publish(optimal_rollout_array);
  optimal_rollout_des_publisher_.publish(optimal_rollout_array_des);
  optimal_linear_input_publisher_.publish(optimal_inputs_lin_array);
  optimal_angular_input_publisher_.publish(optimal_inputs_ang_array);
  optimal_rollout_lin_vel_.publish(optimal_rollout_lin_vel);
  optimal_rollout_ang_vel_.publish(optimal_rollout_ang_vel);

  mppi_reference.position.x = ref_.rr[0](0);
  mppi_reference.position.y = ref_.rr[0](1);
  mppi_reference.position.z = ref_.rr[0](2);
  mppi_reference.orientation.w = ref_.rr[0](3);
  mppi_reference.orientation.x = ref_.rr[0](4);
  mppi_reference.orientation.y = ref_.rr[0](5);
  mppi_reference.orientation.z = ref_.rr[0](6);
  mppi_reference_publisher_.publish(mppi_reference);

  if (task_ == InteractionTask::Shelf && detailed_publishing_) {
    publishShelfInfo(header);
  } else if (task_ == InteractionTask::Valve && detailed_publishing_) {
    publishHookPos(header);
  }
}

void OMAVControllerInterface::publishHookPos(
    const std_msgs::Header &header) const {
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
  for (size_t i = 0; i < xx_opt_.size(); i++) {
    cost_valve_->compute_cost(xx_opt_[i], ref_.rr[0], i * 0.015);
    point.x = cost_valve_->hook_pos_(0);
    point.y = cost_valve_->hook_pos_(1);
    point.z = cost_valve_->hook_pos_(2);
    color.r = static_cast<float>(i) / xx_opt_.size();
    color.g = 1.0f - static_cast<float>(i) / xx_opt_.size();

    marker_hook.points.push_back(point);
    marker_hook.colors.push_back(color);
  }
  pub_marker_hook_.publish(marker_hook);
}

void OMAVControllerInterface::publishShelfInfo(
    const std_msgs::Header &header) const {
  // Info about costs:
  float velocity_cost = 0.0f;
  float power_cost = 0.0f;
  float object_cost = 0.0f;
  float torque_cost = 0.0f;
  float handle_hook_cost = 0.0f;
  float pose_cost = 0.0f;
  float overall_cost = 0.0f;

  visualization_msgs::MarkerArray force_marker_array;

  visualization_msgs::Marker force_marker;
  conversions::arrow_initialization(force_marker);
  force_marker.header = header;

  for (size_t i = 0; i < xx_opt_.size(); i++) {
    // Cost publishing
    cost_shelf_->compute_cost(xx_opt_[i], ref_.rr[0], i * 0.015);
    // Velocity Cost
    velocity_cost += cost_shelf_->cost_vector_(CostIdx::velocity);
    // Efficiency Cost
    power_cost += cost_shelf_->cost_vector_(CostIdx::efficiency);
    // Object Cost
    object_cost += cost_shelf_->cost_vector_(CostIdx::object);
    // Torque Cost
    torque_cost += cost_shelf_->cost_vector_(CostIdx::torque);
    // Handle Hook Cost
    handle_hook_cost += cost_shelf_->cost_vector_(CostIdx::handle_hook);
    // Pose Cost
    pose_cost += cost_shelf_->cost_vector_(CostIdx::pose);
    overall_cost += cost_shelf_->cost_;

    Eigen::Vector3d force_normed = xx_opt_[i].segment<3>(15).normalized();

    force_marker.points[0].x = cost_shelf_->hook_pos_(0);
    force_marker.points[0].y = cost_shelf_->hook_pos_(1);
    force_marker.points[0].z = cost_shelf_->hook_pos_(2);
    force_marker.points[1].x = cost_shelf_->hook_pos_(0) + force_normed(0);
    force_marker.points[1].y = cost_shelf_->hook_pos_(1) + force_normed(1);
    force_marker.points[1].z = cost_shelf_->hook_pos_(2) + force_normed(2);
    force_marker.id = i;

    force_marker_array.markers.push_back(force_marker);
  }
  normalized_force_publisher_.publish(force_marker_array);

  mppi_ros::Array cost_array_message;
  cost_array_message.array.push_back(velocity_cost);
  cost_array_message.array.push_back(power_cost);
  cost_array_message.array.push_back(object_cost);
  cost_array_message.array.push_back(torque_cost);
  cost_array_message.array.push_back(handle_hook_cost);
  cost_array_message.array.push_back(xx_opt_[0](15));
  cost_array_message.array.push_back(xx_opt_[0](16));
  cost_array_message.array.push_back(xx_opt_[0](17));
  cost_array_message.array.push_back(xx_opt_[0].segment<3>(15).norm());
  cost_array_message.array.push_back(
      xx_opt_[1].segment<3>(15).cross(com_hook_)(0));
  cost_array_message.array.push_back(
      xx_opt_[1].segment<3>(15).cross(com_hook_)(1));
  cost_array_message.array.push_back(
      xx_opt_[1].segment<3>(15).cross(com_hook_)(2));
  cost_array_message.array.push_back(
      xx_opt_[1].segment<3>(15).cross(com_hook_).norm());
  cost_array_message.array.push_back(
      acos(xx_opt_[1].segment<3>(15).normalized().dot(com_hook_.normalized())) *
      180.0 / M_PI);
  cost_array_message.array.push_back(pose_cost);
  cost_array_message.array.push_back(overall_cost);

  cost_publisher_.publish(cost_array_message);
}

void OMAVControllerInterface::setDampingFactor(const double &d) {
  damping_ = d;
}

void OMAVControllerInterface::manually_shift_input(const int &index) {
  // deprecated
  if (index != 0) {
    get_controller()->shift_input_ = true;
  }
  // Set the shift variable of the MPPI
  get_controller()->shift_int_ = index;
  get_controller()->first_mppi_iteration_ = false;
}

// namespace omav_interaction
