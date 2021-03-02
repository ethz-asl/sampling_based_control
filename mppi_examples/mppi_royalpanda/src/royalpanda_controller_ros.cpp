//
// Created by giuseppe on 22.01.21.
//

#include <mppi_royalpanda/royalpanda_controller_ros.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

#include <manipulation_msgs/conversions.h>

namespace royalpanda {

bool RoyalPandaControllerRos::init(hardware_interface::RobotHW* robot_hw,
                                   ros::NodeHandle& node_handle) {
  if (!init_parameters(node_handle)) return false;
  if (!init_common_interfaces(robot_hw)) return false;
  if (!init_franka_interfaces(robot_hw)) return false;
  if (!init_ros(node_handle)) return false;

  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);

  if (sim_) {
    std::string arm_description;
    if (!node_handle.getParam("/arm_description", arm_description)) {
      ROS_ERROR("Could not find arm_description on the param server.");
      return false;
    }
    robot_model_ = std::make_unique<rc::RobotWrapper>();
    robot_model_->initFromXml(arm_description);
  } else {
    man_interface_ =
        std::make_unique<manipulation::PandaControllerInterface>(node_handle);
    if (!man_interface_->init()) {
      ROS_ERROR("Failed to initialized manipulation interface");
      return false;
    }
    ROS_INFO("Solver initialized correctly.");
  }

  ROS_INFO("Controller successfully initialized!");
  started_ = false;
  return true;
}

bool RoyalPandaControllerRos::init_parameters(ros::NodeHandle& node_handle) {
  if (!node_handle.getParam("simulation", sim_)) {
    ROS_ERROR("RoyalPandaControllerRos: Could not read parameter simulation");
    return false;
  }

  if (!node_handle.getParam("fixed_base", fixed_base_)) {
    ROS_ERROR("RoyalPandaControllerRos: Could not read parameter fixed_base");
    return false;
  }

  if (!node_handle.getParam("arm_id", arm_id_)) {
    ROS_ERROR("RoyalPandaControllerRos: Could not read parameter arm_id");
    return false;
  }

  if (!node_handle.getParam("joint_names", joint_names_) ||
      joint_names_.size() != 7) {
    ROS_ERROR(
        "RoyalPandaControllerRos: Invalid or no joint_names parameters "
        "provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("i_gains", i_gains_) || i_gains_.size() != 7) {
    ROS_ERROR(
        "RoyalPandaControllerRos:  Invalid or no i_gain parameters provided, "
        "aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
    ROS_ERROR(
        "RoyalPandaControllerRos:  Invalid or no d_gain parameters provided, "
        "aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("arm_I_max", arm_I_max_) || arm_I_max_ < 0) {
    ROS_ERROR(
        "RoyalPandaControllerRos:  Invalid or no arm_I_max parameters "
        "provided, aborting "
        "controller init!");
    return false;
  }

  std::vector<double> base_K_gains;
  if (!node_handle.getParam("base_K_gains", base_K_gains) ||
      base_K_gains.size() != 3) {
    ROS_ERROR(
        "RoyalPandaControllerRos:  Invalid or no base_K_gains parameters "
        "provided, aborting "
        "controller init!");
    return false;
  }
  base_K_gains_ << base_K_gains[0], base_K_gains[1], base_K_gains[2];

  std::vector<double> base_I_gains;
  if (!node_handle.getParam("base_I_gains", base_I_gains) ||
      base_I_gains.size() != 3) {
    ROS_ERROR(
        "RoyalPandaControllerRos:  Invalid or no base_I_gains parameters "
        "provided, aborting "
        "controller init!");
    return false;
  }
  base_I_gains_ << base_I_gains[0], base_I_gains[1], base_I_gains[2];

  if (!node_handle.getParam("I_max", I_max_) || I_max_ < 0) {
    ROS_ERROR(
        "RoyalPandaControllerRos:  Invalid or no I_max parameters provided, "
        "aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("base_filter_alpha", base_filter_alpha_) ||
      (base_filter_alpha_ < 0) || (base_filter_alpha_ > 1)) {
    ROS_ERROR_STREAM(
        "RoyalPandaControllerRos:  Invalid or no base_filter_alpha parameters "
        "provided, aborting "
        "controller init!"
        << base_filter_alpha_);
    return false;
  }

  if (!node_handle.getParam("state_topic", state_topic_)) {
    ROS_INFO_STREAM("RoyalPandaControllerRos: state_topic not found");
    return false;
  }

  if (!node_handle.getParam("base_twist_topic", base_twist_topic_)) {
    ROS_INFO_STREAM("RoyalPandaControllerRos: base_twist_topic not found");
    return false;
  }

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM(
        "RoyalPandaControllerRos: publish_rate not found. Defaulting to "
        << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM(
        "RoyalPandaControllerRos: coriolis_factor not found. Defaulting to "
        << coriolis_factor_);
  }

  ROS_INFO(
      "[RoyalPandaControllerRos::init_parameters]: Parameters successfully "
      "initialized.");
  return true;
}

bool RoyalPandaControllerRos::init_common_interfaces(
    hardware_interface::RobotHW* robot_hw) {
  // Effort interface for compliant control
  auto* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "RoyalPandaControllerRos: Error getting effort joint interface from "
        "hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(
          effort_joint_interface->getHandle(joint_names_[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "RoyalPandaControllerRos: Exception getting joint handles: "
          << ex.what());
      return false;
    }
  }
  ROS_INFO(
      "[RoyalPandaControllerRos::init_common_interfaces]: Interfeces "
      "successfully initialized.");
  return true;
}

bool RoyalPandaControllerRos::init_franka_interfaces(
    hardware_interface::RobotHW* robot_hw) {
  if (sim_) return true;

  // Model interface: returns kino-dynamic properties of the manipulator
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "RoyalPandaControllerRos: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id_ + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "RoyalPandaControllerRos: Exception getting model handle from "
        "interface: "
        << ex.what());
    return false;
  }

  // Get state interface.
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "RoyalPandaControllerRos: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id_ + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "RoyalPandaControllerRos: Exception getting state handle from "
        "interface: "
        << ex.what());
    return false;
  }
  ROS_INFO(
      "[RoyalPandaControllerRos::iinit_franka_interfaces]: Interfaces "
      "successfully initialized.");
  return true;
}

bool RoyalPandaControllerRos::init_ros(ros::NodeHandle& node_handle) {
  input_state_subscriber_ = node_handle.subscribe(
      "/controller/input_state", 1,
      &RoyalPandaControllerRos::input_state_callback, this);
  torques_publisher_.init(node_handle, "torque_comparison", 1);
  base_twist_publisher_.init(node_handle, base_twist_topic_, 1);
  state_subscriber_ = node_handle.subscribe(
      state_topic_, 1, &RoyalPandaControllerRos::state_callback, this);
  ROS_INFO_STREAM("Sending base commands to: " << base_twist_topic_ << std::endl
                                               << "Receiving state at: "
                                               << state_topic_);
  nominal_state_publisher_.init(node_handle, "/x_nom", 1);
  world_twist_publisher_ =
      node_handle.advertise<geometry_msgs::TwistStamped>("/base_twist", 1);
  ROS_INFO(
      "[RoyalPandaControllerRos::init_ros]: Ros successfully initialized.");
  return true;
}

void RoyalPandaControllerRos::state_callback(
    const manipulation_msgs::StateConstPtr& state_msg) {
  if (!started_) return;

  manipulation::conversions::msgToEigen(*state_msg, x_);
  x_ros_ = *state_msg;

  if (!sim_) {
    man_interface_->set_observation(x_, ros::Time::now().toSec());
  }

  if (!state_received_) {
    state_received_ = true;
    state_last_receipt_time_ = ros::Time::now().toSec();
  } else {
    double current_time = ros::Time::now().toSec();
    double elapsed = current_time - state_last_receipt_time_;
    if (elapsed > 0.01) {
      state_ok_ = false;
      ROS_DEBUG_STREAM_THROTTLE(2.0,
                                "[RoyalPandaControllerRos::state_callback] "
                                "State update is too slow. Current delay: "
                                    << elapsed);
    }
    state_last_receipt_time_ = current_time;
  }
}

void RoyalPandaControllerRos::input_state_callback(
    const manipulation_msgs::InputStateConstPtr& msg) {
  manipulation::conversions::msgToEigen(msg->state, x_nom_);
  manipulation::conversions::msgToEigen(msg->input, u_);
}

void RoyalPandaControllerRos::starting(const ros::Time& time) {
  if (started_) return;

  if (!started_ && !sim_) {
    if (!man_interface_->start()) {
      ROS_ERROR(
          "[RoyalPandaControllerRos::starting] Failed  to start controller");
      started_ = false;
      return;
    }
  }

  // integral behavior
  for (size_t i = 0; i < 7; i++) {
    qd_[i] = joint_handles_[i].getPosition();
  }
  base_integral_error_.setZero();
  arm_integral_error_.setZero();

  started_ = true;
  ROS_INFO("[RoyalPandaControllerRos::starting] Controller started!");
}

void RoyalPandaControllerRos::send_command_base(const ros::Duration& period) {
  if (base_trigger_() && base_twist_publisher_.trylock()) {
    Eigen::Vector3d twist_nominal(x_nom_ros_.base_twist.linear.x,
                                  x_nom_ros_.base_twist.linear.y,
                                  x_nom_ros_.base_twist.angular.z);

    Eigen::Matrix3d r_world_base;
    double theta = x_ros_.base_pose.z;
    r_world_base << std::cos(theta), -std::sin(theta), 0.0, std::sin(theta),
        std::cos(theta), 0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix3d r_base_world = r_world_base.transpose();
    Eigen::Vector3d twist_cmd = r_base_world * twist_nominal;

    base_twist_publisher_.msg_.linear.x = u_[0];   // twist_cmd.x();
    base_twist_publisher_.msg_.linear.y = u_[1];   // twist_cmd.y();
    base_twist_publisher_.msg_.angular.z = u_[2];  // twist_cmd.z();
    base_twist_publisher_.unlockAndPublish();

    twist_stamped_.twist = x_nom_ros_.base_twist;
    twist_stamped_.header.frame_id = "world";
    twist_stamped_.header.stamp = ros::Time::now();
    world_twist_publisher_.publish(twist_stamped_);
  }
}

void RoyalPandaControllerRos::send_command_base_from_position(
    const ros::Duration& period) {
  if (base_trigger_() && base_twist_publisher_.trylock()) {
    Eigen::Vector3d position_current(x_ros_.base_pose.x, x_ros_.base_pose.y,
                                     x_ros_.base_pose.z);

    Eigen::Vector3d position_nominal(
        x_nom_ros_.base_pose.x, x_nom_ros_.base_pose.y, x_nom_ros_.base_pose.z);

    Eigen::Matrix3d r_base_world;
    double theta = x_ros_.base_pose.z;
    r_base_world << std::cos(theta), std::sin(theta), 0.0, -std::sin(theta),
        std::cos(theta), 0.0, 0.0, 0.0, 1.0;

    base_position_error_ = r_base_world * (position_nominal - position_current);

    // compute integral and cap error
    base_integral_error_ += base_position_error_ * 0.02;
    base_integral_error_ =
        base_integral_error_.cwiseMax(-I_max_).cwiseMin(I_max_);

    twist_cmd_ = base_K_gains_.cwiseProduct(base_position_error_) +
                 base_I_gains_.cwiseProduct(base_integral_error_);
    ROS_INFO_STREAM_THROTTLE(
        2.0, "Integral error is: "
                 << base_integral_error_.transpose() << std::endl
                 << "Position error is: " << base_position_error_.transpose()
                 << std::endl
                 << "Twist cmd is:" << twist_cmd_.transpose());

    // twist_cmd_.x() = std::abs(twist_cmd_.x()) > 0.01 ? twist_cmd_.x() : 0.0;
    // twist_cmd_.y() = std::abs(twist_cmd_.y()) > 0.01 ? twist_cmd_.y() : 0.0;
    // twist_cmd_.z() = std::abs(twist_cmd_.z()) > 0.01 ? twist_cmd_.z() : 0.0;

    base_twist_publisher_.msg_.linear.x = twist_cmd_.x();
    base_twist_publisher_.msg_.linear.y = twist_cmd_.y();
    base_twist_publisher_.msg_.angular.z = twist_cmd_.z();
    base_twist_publisher_.unlockAndPublish();

    twist_stamped_.twist = x_nom_ros_.base_twist;
    twist_stamped_.header.frame_id = "world";
    twist_stamped_.header.stamp = ros::Time::now();
    world_twist_publisher_.publish(twist_stamped_);
  }
}

void RoyalPandaControllerRos::send_command_arm(const ros::Duration& period) {
  if (sim_) {
    Eigen::Matrix<double, 9, 1> q = Eigen::Matrix<double, 9, 1>::Zero();
    Eigen::Matrix<double, 9, 1> v = Eigen::Matrix<double, 9, 1>::Zero();
    for (size_t i = 0; i < 7; i++) {
      q[i] = joint_handles_[i].getPosition();
      v[i] = joint_handles_[i].getVelocity();
    }

    robot_model_->updateState(q, v);
    robot_model_->computeAllTerms();
    Eigen::VectorXd tau = robot_model_->getNonLinearTerms();

    for (size_t i = 0; i < 7; i++) {
      qd_[i] += u_.tail<8>()(i) * period.toSec();
      tau[i] += i_gains_[i] * (qd_[i] - q[i]) +
                d_gains_[i] * (u_.tail<8>()[i] - v[i]);
      joint_handles_[i].setCommand(tau[i]);
    }
  } else {
    robot_state_ = state_handle_->getRobotState();
    std::array<double, 7> coriolis = model_handle_->getCoriolis();
    std::array<double, 7> gravity = model_handle_->getGravity();

    double alpha = 0.99;
    for (size_t i = 0; i < 7; i++) {
      dq_filtered_[i] =
          (1 - alpha) * dq_filtered_[i] + alpha * robot_state_.dq[i];
    }

    std::array<double, 7> tau_d_calculated{};
    for (size_t i = 0; i < 7; ++i) {
      qd_[i] += u_.tail<8>()(i) * period.toSec();
      arm_integral_error_[i] =
          (x_nom_ros_.arm_state.position[i] - x_ros_.arm_state.position[i]) *
          period.toSec();
      arm_integral_error_[i] =
          std::max(std::min(arm_integral_error_[i], arm_I_max_), -arm_I_max_);

      tau_d_calculated[i] = coriolis_factor_ * coriolis[i];
      tau_d_calculated[i] = coriolis_factor_ * coriolis[i] +
                            i_gains_[i] * (qd_[i] - robot_state_.q[i]) +
                            // i_gains_[i] * arm_integral_error_[i] +
                            d_gains_[i] * (u_.tail<8>()(i) - dq_filtered_[i]);
    }

    // Maximum torque difference with a sampling rate of 1 kHz. The maximum
    // torque rate is 1000 * (1 / sampling_time).
    std::array<double, 7> tau_d_saturated =
        saturateTorqueRate(tau_d_calculated, robot_state_.tau_J_d);

    for (size_t i = 0; i < 7; ++i) {
      joint_handles_[i].setCommand(tau_d_saturated[i]);
    }
  }
}

void RoyalPandaControllerRos::update(const ros::Time& time,
                                     const ros::Duration& period) {
  if (!started_) {
    ROS_ERROR_ONCE(
        "[RoyalPandaControllerRos::update]: Controller not started. Probably "
        "some error "
        "occourred...");
    return;
  }

  if (!state_received_) {
    ROS_WARN_STREAM_THROTTLE(
        2.0, "[RoyalPandaControllerRos::update]State not received yet.");
    return;
  }

  if (!sim_) {
    man_interface_->get_input_state(x_, x_nom_, u_, time.toSec());
  }

  manipulation::conversions::eigenToMsg(x_nom_, x_nom_ros_);

  send_command_arm(period);
  if (!fixed_base_) send_command_base(period);

  if (nominal_state_publisher_.trylock()) {
    nominal_state_publisher_.msg_ = x_nom_ros_;
    nominal_state_publisher_.unlockAndPublish();
  }
}

void RoyalPandaControllerRos::stopping(const ros::Time& time) {}

std::array<double, 7> RoyalPandaControllerRos::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>&
        tau_J_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] +
        std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}  // namespace royalpanda

PLUGINLIB_EXPORT_CLASS(royalpanda::RoyalPandaControllerRos,
                       controller_interface::ControllerBase)