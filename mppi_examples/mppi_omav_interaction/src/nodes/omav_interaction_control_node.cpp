
#include <mppi_omav_interaction/nodes/omav_interaction_control_node.h>

namespace omav_interaction {

InteractionControlNode::InteractionControlNode(ros::NodeHandle &nh,
                                               ros::NodeHandle &private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      controller_(private_nh, nh),
      cost_shelf_param_server_(
          ros::NodeHandle(private_nh, "cost_shelf_parameters")),
      cost_valve_param_server_(
          ros::NodeHandle(private_nh, "cost_valve_parameters")),
      mppiSettingsParamServer_(ros::NodeHandle(private_nh, "mppi_settings")) {
  // timed_command_ = nh_.createTimer(
  //     ros::Duration(1. / kControl_rate),
  //     &InteractionControlNode::TimedCommandCallback, this, false, true);

  // load parameters
  InitializeNodeParams();
}

InteractionControlNode::~InteractionControlNode() {}

bool InteractionControlNode::InitializeNodeParams() {
  // Load ROS parameters
  std::vector<double> x0;
  std::string object_name;
  getParam<bool>(private_nh_, "running_rotors", running_rotors_, true);
  getParam<std::string>(private_nh_, "object_name", object_name, "shelf");
  getParam<std::vector<double>>(private_nh_, "initial_configuration", x0, {});
  // getParam<bool>(private_nh_, "sequential", sequential_, false);
  if (x0.size() != omav_state_description::SIZE_OMAV_STATE) {
    ROS_ERROR(
        "[mppi_omav_interaction] Wrong size of initial state. Shutting down.");
    ros::shutdown();
  }

  ROS_INFO_STREAM(
      "[mppi_omav_interaction] Robot & Object Description Raisim Loaded");

  controller_.setTask(object_name);

  // set initial state
  // state_ = observation_t::Zero(simulation_->get_state_dimension());
  state_ = Eigen::Matrix<double, omav_state_description::SIZE_OMAV_STATE, 1>(
      x0.data());
  // simulation_->reset(state_);

  // init the controller
  if (!controller_.init())
    throw std::runtime_error("Failed to initialize controller");
  controller_.set_observation(state_, 0.0);
  ROS_INFO_STREAM("[InteractionControlNode] First Odometry received");

  // Set first odometry value as reference
  if (running_rotors_) {
    getState(state_);
    controller_.set_initial_reference(state_);
    initialize_integrators(state_);
  }

  initializeSubscribers();

  // setup dynamic reconfigure
  dynamic_reconfigure::Server<
      mppi_omav_interaction::MPPIOmavSettingsConfig>::CallbackType k;
  k = boost::bind(&InteractionControlNode::mppiSettingsParamCallback, this, _1,
                  _2);
  mppiSettingsParamServer_.setCallback(k);

  if (controller_.getTask() == InteractionTask::Shelf) {
    dynamic_reconfigure::Server<
        mppi_omav_interaction::MPPIOmavCostShelfConfig>::CallbackType g;
    g = boost::bind(&InteractionControlNode::costShelfParamCallback, this, _1,
                    _2);
    cost_shelf_param_server_.setCallback(g);
  } else {
    dynamic_reconfigure::Server<
        mppi_omav_interaction::MPPIOmavCostValveConfig>::CallbackType h;
    h = boost::bind(&InteractionControlNode::costValveParamCallback, this, _1,
                    _2);
    cost_valve_param_server_.setCallback(h);
  }

  return true;
}

void InteractionControlNode::initializeSubscribers() {
  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                &InteractionControlNode::odometryCallback, this,
                                ros::TransportHints().tcpNoDelay());
  object_state_sub_ = nh_.subscribe(
      "object_joint_states", 1, &InteractionControlNode::objectStateCallback,
      this, ros::TransportHints().tcpNoDelay());
  object_pose_subscriber_ = nh_.subscribe(
      "object_pose", 1, &InteractionControlNode::objectPoseCallback, this);
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

void InteractionControlNode::odometryCallback(
    const nav_msgs::OdometryConstPtr &odometry_msg) {
  mav_msgs::eigenOdometryFromMsg(*odometry_msg, &current_odometry_);
  odometry_valid_ = true;
  computeCommand(odometry_msg->header.stamp);

  ROS_INFO_ONCE("[mppi_omav_interaction] MPPI got odometry message");
}

void InteractionControlNode::objectStateCallback(
    const sensor_msgs::JointState &object_msg) {
  object_state_time_ = object_msg.header.stamp;
  object_state_(object_state_description::OBJECT_HINGE_ORIENTATION) =
      object_msg.position[0];
  object_state_(object_state_description::OBJECT_HINGE_VELOCITY) =
      object_msg.velocity[0];
  object_valid_ = true;
  ROS_INFO_ONCE("[mppi_omav_interaction] MPPI got first object state message");
}

void InteractionControlNode::objectPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped &pose_msg) {
  conversions::vectorFromPoseMsg(pose_msg.pose.pose, object_pose_);
  ROS_INFO_ONCE("[mppi_omav_interaction] MPPI got first object pose message");
}

// void InteractionControlNode::TimedCommandCallback(const ros::TimerEvent &e) {
//   computeCommand(e.current_real);
// }

bool InteractionControlNode::computeCommand(const ros::Time &t_now) {
  if (!controller_running_) {
    // Start controller once odometry has been received
    controller_.start();
    controller_running_ = true;
  }

  if (controller_.getTask() == InteractionTask::Valve) {
    // if (state_(omav_state_description::OBJECT_HINGE_ORIENTATION) +
    // cost_valve_params_.ref_p > last_ref) { last_ref =
    // state_(omav_state_description::OBJECT_HINGE_ORIENTATION) +
    // cost_valve_params_.ref_p; controller_.updateValveReference(last_ref); Use
    // dynamic updating of the valve reference: Reference angle increases
    // throughout the horizon
    if (cost_valve_params_.cost_mode == 0) {
      controller_.updateValveReferenceDynamic(
          state_(omav_state_description::OBJECT_HINGE_ORIENTATION) +
              cost_valve_params_.ref_p,
          state_(omav_state_description::OBJECT_HINGE_ORIENTATION) +
              cost_valve_params_.ref_p + cost_valve_params_.ref_v,
          t_now.toSec());
    } else {
      controller_.updateValveReference(cost_valve_params_.ref_angle);
    }
    // }
  }

  // Load most recently published trajectory
  if (controller_.get_current_trajectory(&current_trajectory_)) {
    trajectory_available_ = true;
  }

  // Get recent state by combining odometry, object, and recently published
  // trajectory
  getState(state_);

  const double t_odom =
      1.e-9 * static_cast<double>(current_odometry_.timestamp_ns);
  controller_.set_observation(state_, t_odom);
  return true;
}

bool InteractionControlNode::getState(observation_t &x) {
  if (!odometry_valid_ || !object_valid_) {
    return false;
  }
  // Synchronize current target with recent odometry
  getTargetStateFromTrajectory();

  x.head<3>() = current_odometry_.position_W;
  x(omav_state_description::MAV_ORIENTATION_W_WORLD) =
      current_odometry_.orientation_W_B.w();
  x.segment<3>(omav_state_description::MAV_ORIENTATION_X_WORLD) =
      current_odometry_.orientation_W_B.vec();
  x.segment<3>(omav_state_description::MAV_LINEAR_VELOCITY_X_WORLD) =
      current_odometry_.getVelocityWorld();
  x.segment<3>(omav_state_description::MAV_ANGULAR_VELOCITY_X_BODY) =
      current_odometry_.angular_velocity_B;
  x.segment<2>(omav_state_description::OBJECT_HINGE_ORIENTATION) =
      object_state_;
  x.segment<3>(omav_state_description::MAV_POSITION_X_DESIRED_WORLD) =
      target_state_.position_W;
  x(omav_state_description::MAV_ORIENTATION_W_DESIRED_WORLD) =
      target_state_.orientation_W_B.w();
  x.segment<3>(omav_state_description::MAV_ORIENTATION_X_DESIRED_WORLD) =
      target_state_.orientation_W_B.vec();
  x.segment<3>(omav_state_description::MAV_LINEAR_VELOCITY_X_DESIRED_WORLD) =
      target_state_.velocity_W;
  x.segment<3>(omav_state_description::MAV_ANGULAR_VELOCITY_X_DESIRED_BODY) =
      target_state_.angular_velocity_W;
  x.segment<7>(omav_state_description::OBJECT_POSITION_X_WORLD) = object_pose_;

  ROS_INFO_ONCE("[mppi_omav_interaction] MPPI got first state message");
  if (ros::Time::now() - object_state_time_ > ros::Duration(0.5)) {
    ROS_WARN_THROTTLE(0.5,
                      "[mppi_omav_interaction] Object state timeout: t = %fs",
                      (ros::Time::now() - object_state_time_).toSec());
  }
  return true;
}

bool InteractionControlNode::getTargetStateFromTrajectory() {
  // Compute target state according to odometry timestamp:
  if (trajectory_available_ && current_trajectory_.points.size() > 0) {
    const int64_t t_stamp = current_trajectory_.header.stamp.toNSec();
    const int64_t t_odom_s = current_odometry_.timestamp_ns;
    int64_t t_ref = t_stamp;
    size_t i = 0;
    while (t_ref < t_odom_s && i < current_trajectory_.points.size()) {
      t_ref = t_stamp + current_trajectory_.points[i].time_from_start.toNSec();
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
      double t = static_cast<double>((t_odom_s - (t_ref - kSim_dt_ns)) / 1e9) /
                 kSim_dt;
      mav_msgs::EigenTrajectoryPoint target_state;
      omav_interaction::conversions::InterpolateTrajectoryPoints(
          current_trajectory_.points[i - 2], current_trajectory_.points[i - 1],
          t, &target_state);
      setTarget(target_state);
    } else {
      setTarget(current_trajectory_.points[i]);
      ROS_WARN("Was not able to interpolate trajectory points.");
    }
    return true;
  }
  // If there's no trajectory available, use current odometry as default target
  // state. This is just to ensure that the solver does not diverge.
  target_state_.position_W = current_odometry_.position_W;
  target_state_.orientation_W_B = current_odometry_.orientation_W_B;
  target_state_.velocity_W = Eigen::Vector3d::Zero();
  target_state_.angular_velocity_W = Eigen::Vector3d::Zero();
  ROS_ERROR(
      "No trajectory available or length 0 (%i, %i). Using odometry as target.",
      static_cast<int>(trajectory_available_),
      static_cast<int>(current_trajectory_.points.size()));
  return false;
}

bool InteractionControlNode::setTarget(
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

bool InteractionControlNode::setTarget(
    const mav_msgs::EigenTrajectoryPoint &target_state) {
  target_state_ = target_state;
  return true;
}

bool InteractionControlNode::initialize_integrators(observation_t &x) {
  if (!odometry_valid_) {
    return false;
  }
  x.segment<3>(omav_state_description::MAV_POSITION_X_DESIRED_WORLD) =
      current_odometry_.position_W;
  x(omav_state_description::MAV_ORIENTATION_W_DESIRED_WORLD) =
      current_odometry_.orientation_W_B.w();
  x.segment<3>(omav_state_description::MAV_ORIENTATION_X_DESIRED_WORLD) =
      current_odometry_.orientation_W_B.vec();
  x.segment<3>(omav_state_description::MAV_LINEAR_VELOCITY_X_DESIRED_WORLD) =
      current_odometry_.getVelocityWorld();
  x.segment<3>(omav_state_description::MAV_ANGULAR_VELOCITY_X_DESIRED_BODY) =
      current_odometry_.angular_velocity_B;

  target_state_.position_W = current_odometry_.position_W;
  target_state_.orientation_W_B = current_odometry_.orientation_W_B;
  target_state_.velocity_W = current_odometry_.getVelocityWorld();
  target_state_.angular_velocity_W = current_odometry_.angular_velocity_B;
  return true;
}

void InteractionControlNode::costShelfParamCallback(
    mppi_omav_interaction::MPPIOmavCostShelfConfig &config, uint32_t level) {
  if (controller_.getTask() == InteractionTask::Shelf) {
    controller_.getCostParamShelf(cost_shelf_params_);
    // Update OMAV pose cost
    cost_shelf_params_.Q_x_omav = config.Q_x_omav;
    cost_shelf_params_.Q_y_omav = config.Q_y_omav;
    cost_shelf_params_.Q_z_omav = config.Q_z_omav;
    cost_shelf_params_.Q_orientation = config.Q_orientation_omav;

    cost_shelf_params_.pose_costs << cost_shelf_params_.Q_x_omav,
        cost_shelf_params_.Q_y_omav, cost_shelf_params_.Q_z_omav,
        cost_shelf_params_.Q_orientation, cost_shelf_params_.Q_orientation,
        cost_shelf_params_.Q_orientation;
    cost_shelf_params_.Q_pose = cost_shelf_params_.pose_costs.asDiagonal();

    cost_shelf_params_.pose_costs_int << 0, 0, 0, config.Q_int_roll,
        config.Q_int_pitch, 0;
    cost_shelf_params_.Q_pose_int =
        cost_shelf_params_.pose_costs_int.asDiagonal();

    // Update Object cost
    cost_shelf_params_.Q_object = config.Q_object;
    // Update velocity cost
    cost_shelf_params_.Q_lin_vel = config.Q_linear_velocity;
    cost_shelf_params_.vel_costs << cost_shelf_params_.Q_lin_vel,
        cost_shelf_params_.Q_lin_vel, cost_shelf_params_.Q_lin_vel,
        config.Q_roll, config.Q_pitch, config.Q_yaw;
    cost_shelf_params_.Q_vel = cost_shelf_params_.vel_costs.asDiagonal();
    // Update Handle Hook cost components
    cost_shelf_params_.handle_hook_thresh = config.handle_hook_thresh;
    cost_shelf_params_.Q_handle_hook = config.handle_hook_cost;
    // Update floor cost components
    cost_shelf_params_.floor_thresh = config.floor_thresh;
    cost_shelf_params_.Q_force = config.force_cost;
    cost_shelf_params_.Q_efficiency = config.efficiency_cost;
    cost_shelf_params_.Q_floor = config.floor_cost;
    cost_shelf_params_.Q_torque = config.torque_cost;
    cost_shelf_params_.contact_bool = config.contact_prohibitor;
    controller_.update_cost_param_shelf(cost_shelf_params_);
    std::cout << "Updated costs." << std::endl
              << cost_shelf_params_ << std::endl;
  } else {
    ROS_ERROR("Wrong scenario chosen. Cannot set cost.");
  }
}

void InteractionControlNode::mppiSettingsParamCallback(
    mppi_omav_interaction::MPPIOmavSettingsConfig &config, uint32_t level) {
  Eigen::Matrix<double, 6, 1> dGains;
  dGains << config.d_gain_pos * Eigen::Vector3d::Ones(),
      config.d_gain_ang * Eigen::Vector2d::Ones(), config.d_gain_yaw;

  std::vector<std::shared_ptr<OMAVVelocityDynamics>> omav_dynamics_v;
  controller_.getDynamicsPtr(omav_dynamics_v);
  size_t n = omav_dynamics_v.size();
  for (size_t i = 0; i < n; i++) {
    omav_dynamics_v.at(i)->setDGains(dGains);
    omav_dynamics_v.at(i)->setDampingFactor(config.damping);
    omav_dynamics_v.at(i)->setMass(config.mass);
  }
  controller_.setDampingFactor(config.damping);
  controller_.setHoldTime(config.hold_time);
}

void InteractionControlNode::costValveParamCallback(
    mppi_omav_interaction::MPPIOmavCostValveConfig &config, uint32_t level) {
  if (controller_.getTask() == InteractionTask::Valve) {
    controller_.getCostParamValve(cost_valve_params_);
    cost_valve_params_.ref_p = config.ref_p;
    cost_valve_params_.ref_v = config.ref_v;
    cost_valve_params_.Q_x_omav = config.Q_x_omav;
    cost_valve_params_.Q_y_omav = config.Q_y_omav;
    cost_valve_params_.Q_z_omav = config.Q_z_omav;
    cost_valve_params_.Q_orientation = config.Q_orientation_omav;
    cost_valve_params_.pose_costs << cost_valve_params_.Q_x_omav,
        cost_valve_params_.Q_y_omav, cost_valve_params_.Q_z_omav,
        cost_valve_params_.Q_orientation, cost_valve_params_.Q_orientation,
        cost_valve_params_.Q_orientation;
    cost_valve_params_.Q_pose = cost_valve_params_.pose_costs.asDiagonal();

    cost_valve_params_.pose_costs_int << 0, 0, 0, config.Q_int_roll,
        config.Q_int_pitch, 0;
    cost_valve_params_.Q_pose_int =
        cost_valve_params_.pose_costs_int.asDiagonal();

    // Update Object cost
    cost_valve_params_.Q_object = config.Q_object;
    // Update velocity cost
    cost_valve_params_.Q_lin_vel = config.Q_linear_velocity;
    cost_valve_params_.vel_costs << cost_valve_params_.Q_lin_vel,
        cost_valve_params_.Q_lin_vel, cost_valve_params_.Q_lin_vel,
        config.Q_roll, config.Q_pitch, config.Q_yaw;
    cost_valve_params_.Q_vel = cost_valve_params_.vel_costs.asDiagonal();
    // Update Handle Hook cost components
    cost_valve_params_.handle_hook_thresh = config.handle_hook_thresh;
    cost_valve_params_.Q_handle_hook = config.handle_hook_cost;
    // Update floor cost components
    cost_valve_params_.floor_thresh = config.floor_thresh;
    cost_valve_params_.Q_floor = config.floor_cost;
    cost_valve_params_.Q_force = config.force_cost;

    cost_valve_params_.Q_forcev << config.Q_force_x, config.Q_force_y,
        config.Q_force_z;
    // cost_valve_params_.Q_efficiency = config.efficiency_cost;
    // cost_valve_params_.Q_torque = config.torque_cost;
    cost_valve_params_.contact_bool = config.contact_prohibitor;
    cost_valve_params_.Q_unwanted_contact = config.Q_unwanted_contact;
    cost_valve_params_.Q_object_distance = config.Q_object_distance;
    cost_valve_params_.object_distance_thresh = config.object_distance_thresh;
    cost_valve_params_.cost_mode = config.cost_mode;
    cost_valve_params_.ref_angle = config.ref_angle;
    cost_valve_params_.Q_handle_hookV = config.Q_handle_hookV;
    if (config.int_mode == true) {
      controller_.setInteractionMode(interaction_mode::INTERACTION);
    } else {
      controller_.setInteractionMode(interaction_mode::FREE_FLIGHT);
    }
    controller_.update_cost_param_valve(cost_valve_params_);
  } else {
    ROS_ERROR("Wrong scenario chosen. Cannot set cost.");
  }
}

template <class T>
void InteractionControlNode::getParam(const ros::NodeHandle &nh,
                                      const std::string &id, T &var,
                                      const T &val_def) const {
  if (!nh.param<T>(id, var, val_def)) {
    ROS_FATAL_STREAM("[mppi_omav_interaction] Could not find parameter " << id);
    ros::shutdown();
    exit(1);
  } else {
    ROS_INFO_STREAM("[mppi_omav_interaction] Successfully read " << id);
  }
}

}  // namespace omav_interaction

int main(int argc, char **argv) {
  ros::init(argc, argv, "interaction_controller_node");

  ros::NodeHandle nh, private_nh("~");

  // spawn the controller node
  std::shared_ptr<omav_interaction::InteractionControlNode> controller_node(
      new omav_interaction::InteractionControlNode(nh, private_nh));

  ros::spin();

  return 0;
}
