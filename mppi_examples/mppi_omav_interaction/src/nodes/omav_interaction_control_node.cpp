
#include <mppi_omav_interaction/nodes/omav_interaction_control_node.h>

namespace omav_interaction {

InteractionControlNode::InteractionControlNode(ros::NodeHandle &nh,
                                               ros::NodeHandle &private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      controller_(private_nh, nh),
      reference_param_server_(
          ros::NodeHandle(private_nh, "reference_parameters")),
      cost_param_server_(ros::NodeHandle(private_nh, "cost_parameters")),
      cost_valve_param_server_(
          ros::NodeHandle(private_nh, "cost_valve_parameters")) {
  // timed_command_ = nh_.createTimer(
  //     ros::Duration(1. / kControl_rate),
  //     &InteractionControlNode::TimedCommandCallback, this, false, true);

  // load parameters
  InitializeNodeParams();
}

InteractionControlNode::~InteractionControlNode() {}

bool InteractionControlNode::InitializeNodeParams() {
  // Load ROS parameters
  std::string robot_description_raisim;
  std::string object_description;
  std::vector<double> x0;
  std::string object_name;
  getParam<bool>(private_nh_, "running_rotors", running_rotors_, true);
  getParam<std::string>(private_nh_, "object_name", object_name, "shelf");
  getParam<std::string>(private_nh_, "/robot_description_raisim",
                        robot_description_raisim, "");
  getParam<std::string>(private_nh_, "/object_description", object_description,
                        "");
  getParam<std::vector<double>>(private_nh_, "initial_configuration", x0, {});
  // getParam<bool>(private_nh_, "sequential", sequential_, false);

  ROS_INFO_STREAM(
      "[mppi_omav_interaction] Robot & Object Description Raisim Loaded");

  controller_.setTask(object_name);

  // set initial state
  // state_ = observation_t::Zero(simulation_->get_state_dimension());
  state_ = observation_t::Zero(32);
  for (size_t i = 0; i < x0.size(); i++) state_(i) = x0[i];
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

  // Initialize values for the manual timing
  initializeSubscribers();
  initializePublishers();

  // setup dynamic reconfigure
  dynamic_reconfigure::Server<
      mppi_omav_interaction::MPPIOmavReferenceConfig>::CallbackType f;
  dynamic_reconfigure::Server<
      mppi_omav_interaction::MPPIOmavCostConfig>::CallbackType g;
  dynamic_reconfigure::Server<
      mppi_omav_interaction::MPPIOmavCostValveConfig>::CallbackType h;
  f = boost::bind(&InteractionControlNode::referenceParamCallback, this, _1,
                  _2);
  g = boost::bind(&InteractionControlNode::costParamCallback, this, _1, _2);
  h = boost::bind(&InteractionControlNode::costValveParamCallback, this, _1,
                  _2);
  reference_param_server_.setCallback(f);
  cost_param_server_.setCallback(g);
  cost_valve_param_server_.setCallback(h);

  return true;
}

void InteractionControlNode::initializeSubscribers() {
  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                &InteractionControlNode::odometryCallback, this,
                                ros::TransportHints().tcpNoDelay());
  object_state_sub_ = nh_.subscribe("object_joint_states", 1,
                                    &InteractionControlNode::objectCallback,
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

void InteractionControlNode::initializePublishers() {
  reference_publisher_ =
      nh_.advertise<geometry_msgs::PoseStamped>("/mppi_pose_desired", 1);
}

void InteractionControlNode::odometryCallback(
    const nav_msgs::OdometryConstPtr &odometry_msg) {
  mav_msgs::eigenOdometryFromMsg(*odometry_msg, &current_odometry_);
  odometry_valid_ = true;
  computeCommand(odometry_msg->header.stamp);

  ROS_INFO_ONCE("[mppi_omav_interaction] MPPI got odometry message");
}

void InteractionControlNode::objectCallback(
    const sensor_msgs::JointState &object_msg) {
  object_state_(0) = object_msg.position[0];
  object_state_(1) = object_msg.velocity[0];
  object_valid_ = true;
  ROS_INFO_ONCE("[mppi_omav_interaction] MPPI got first object state message");
}

// void InteractionControlNode::TimedCommandCallback(const ros::TimerEvent &e) {
//   computeCommand(e.current_real);
// }

bool InteractionControlNode::computeCommand(const ros::Time &t_now) {
  if (controller_.getTask() == InteractionTask::Valve) {
    // Set valve reference value to current angle
    controller_.updateValveReference(state_(13) + rqt_cost_valve_.ref_p);
  }

  // Load most recently published trajectory
  if (controller_.get_current_trajectory(&current_trajectory_)) {
    trajectory_available_ = true;
  }

  // Get odometry and target state
  ros::Time t_odom_t;
  t_odom_t = t_odom_t.fromNSec(current_odometry_.timestamp_ns);
  const double t_odom = t_odom_t.toSec();
  getState(state_);

  controller_.set_observation(state_, t_odom);
  if (t_odom < 0) {
    ROS_WARN_THROTTLE(
        1.0, "[mppi_omav_interaction] Bad odom time. Is odometry available?");
    return false;
  } else if (!controller_running_) {
    // Start controller once odometry has been received
    controller_.start();
    controller_running_ = true;
  }
  return true;
}

bool InteractionControlNode::getState(observation_t &x) {
  if (!odometry_valid_ || !object_valid_) {
    return false;
  }
  // Synchronize current target with recent odometry
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
  ROS_ERROR("No trajectory available.");
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

void InteractionControlNode::referenceParamCallback(
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
    config.reset_object = false;
    state_(13) = 0;
    // simulation_->reset(state_);
    ROS_INFO("[mppi_omav_interaction] Reset Object");
  }
}

void InteractionControlNode::costParamCallback(
    mppi_omav_interaction::MPPIOmavCostConfig &config, uint32_t level) {
  if (controller_.getTask() == InteractionTask::Shelf) {
    // Update OMAV pose cost
    rqt_cost_shelf_.Q_distance_x = config.Q_x_omav;
    rqt_cost_shelf_.Q_distance_y = config.Q_y_omav;
    rqt_cost_shelf_.Q_distance_z = config.Q_z_omav;
    rqt_cost_shelf_.Q_orientation = config.Q_orientation_omav;
    rqt_cost_shelf_.pose_costs << rqt_cost_shelf_.Q_distance_x,
        rqt_cost_shelf_.Q_distance_y, rqt_cost_shelf_.Q_distance_z,
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
    rqt_cost_shelf_.handle_hook_thresh = config.Handle_Hook_Threshold;
    rqt_cost_shelf_.Q_handle_hook = config.Handle_Hook_Cost;
    // Update floor cost components
    rqt_cost_shelf_.floor_thresh = config.Floor_Threshold;
    rqt_cost_shelf_.Q_floor = config.floor_cost;
    rqt_cost_shelf_.Q_power = config.power_cost;
    rqt_cost_shelf_.Q_torque = config.torque_cost;
    rqt_cost_shelf_.contact_bool = config.contact_prohibitor;
    controller_.update_cost_param_shelf(rqt_cost_shelf_);
  } else {
    ROS_ERROR("Wrong scenario chosen. Cannot set cost.");
  }
}

void InteractionControlNode::costValveParamCallback(
    mppi_omav_interaction::MPPIOmavCostValveConfig &config, uint32_t level) {
  if (controller_.getTask() == InteractionTask::Valve) {
    // Update OMAV pose cost
    Eigen::VectorXd pGains(6);
    Eigen::VectorXd dGains(6);
    pGains << config.p_gain_pos * Eigen::Vector3d::Ones(),
        config.p_gain_ang * Eigen::Vector3d::Ones();
    dGains << config.d_gain_pos * Eigen::Vector3d::Ones(),
        config.d_gain_ang * Eigen::Vector3d::Ones();

    std::vector<std::shared_ptr<OMAVVelocityDynamics>> omav_dynamics_v;
    controller_.getDynamicsPtr(omav_dynamics_v);
    size_t n = omav_dynamics_v.size();
    for (size_t i = 0; i < n; i++) {
      omav_dynamics_v.at(i)->setPDGains(pGains, dGains);
      omav_dynamics_v.at(i)->setDampingFactor(config.damping);
      omav_dynamics_v.at(i)->setMass(config.mass);
    }

    controller_.setDampingFactor(config.damping);
    controller_.setHoldTime(config.hold_time);

    rqt_cost_valve_.ref_p = config.ref_p;
    rqt_cost_valve_.ref_v = config.ref_v;
    rqt_cost_valve_.Q_distance_x = config.Q_x_omav;
    rqt_cost_valve_.Q_distance_y = config.Q_y_omav;
    rqt_cost_valve_.Q_distance_z = config.Q_z_omav;
    rqt_cost_valve_.Q_orientation = config.Q_orientation_omav;
    rqt_cost_valve_.pose_costs << rqt_cost_valve_.Q_distance_x,
        rqt_cost_valve_.Q_distance_y, rqt_cost_valve_.Q_distance_z,
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
    rqt_cost_valve_.handle_hook_thresh = config.Handle_Hook_Threshold;
    rqt_cost_valve_.Q_handle_hook = config.Handle_Hook_Cost;
    // Update floor cost components
    rqt_cost_valve_.floor_thresh = config.Floor_Threshold;
    rqt_cost_valve_.Q_floor = config.floor_cost;
    rqt_cost_valve_.Q_power = config.power_cost;
    rqt_cost_valve_.Q_torque = config.torque_cost;
    rqt_cost_valve_.contact_bool = config.contact_prohibitor;
    controller_.update_cost_param_valve(rqt_cost_valve_);
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
