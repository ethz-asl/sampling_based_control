//
// Created by giuseppe on 16.08.21.
//

#include "mppi_manipulation_royalpanda/simulation.h"
#include "manipulation_msgs/conversions.h"

using namespace manipulation;
using namespace manipulation_royalpanda;

RoyalPandaSim::RoyalPandaSim(const ros::NodeHandle &nh): nh_(nh) {};

bool RoyalPandaSim::init_sim() {
  if (!init_params()){
    ROS_ERROR("Failed to initialize parameters.");
    return false;
  }

  if (!init_dynamics()){
    ROS_ERROR("Failed to initialize dynamics.");
    return false;
  }

  if (!init_handles()){
    ROS_ERROR("Failed to initialize handles.");
    return false;
  }

  return true;
}

bool RoyalPandaSim::init_params() {
  if (!nh_.param<std::vector<std::string>>("arm_joint_names", arm_joint_name_, {}) || arm_joint_name_.size() != 7){
    ROS_ERROR("Failed to parse arm_joint_names");
    return false;
  }
  return true;
}

bool RoyalPandaSim::init_dynamics() {
  bool is_sim = true;
  DynamicsParams params;
  if (!params.init_from_ros(nh_, is_sim)){
    return false;
  }
  dynamics_ = std::make_unique<ManipulatorDynamicsRos>(nh_, params);
  return true;
}

bool RoyalPandaSim::init_handles() {
  arm_joint_position_.setZero(7);
  arm_joint_velocity_.setZero(7);
  arm_joint_effort_.setZero(7);
  arm_joint_effort_desired_.setZero(7);

  // usual interfaces
  for (int i = 0; i < 7; i++) {
    joint_state_if_.registerHandle(hardware_interface::JointStateHandle(
        arm_joint_name_[i], &arm_joint_position_[i], &arm_joint_velocity_[i],
        &arm_joint_effort_[i]));

    effort_joint_if_.registerHandle(hardware_interface::JointHandle(
        joint_state_if_.getHandle(arm_joint_name_[i]), &arm_joint_effort_desired_[i]));
  }

  // franka interfaces
  franka_state_if_.registerHandle(franka_hw::FrankaStateHandle("panda_robot", this->robot_state_));
  franka_model_if_.registerHandle(
      franka_hw::FrankaModelHandle("panda_model", *this->model_, this->robot_state_));

  registerInterface(&joint_state_if_);
  registerInterface(&effort_joint_if_);
  registerInterface(&franka_state_if_);
  registerInterface(&franka_model_if_);
  return true;
}


void RoyalPandaSim::read_sim(ros::Time time, ros::Duration period) {
  manipulation::conversions::fromEigenState(base_position_,
                                    base_twist_,
                                    arm_joint_position_,
                                    arm_joint_velocity_,
                                    object_position_,
                                    object_velocity_,
                                    contact_state_,
                                    tank_state_,
                                    arm_joint_effort_,
                                    dynamics_->get_state());
  for (int i = 0; i < 7; i++) {
    std::string name =  "panda_joint" + std::to_string(i + 1);
    this->robot_state_.q[i] = arm_joint_position_[i];
    this->robot_state_.dq[i] = arm_joint_velocity_[i];
    this->robot_state_.tau_J[i] = arm_joint_effort_[i];
    this->robot_state_.dtau_J[i] = 0.0;

    this->robot_state_.q_d[i] = arm_joint_position_[i];
    this->robot_state_.dq_d[i] = arm_joint_velocity_[i];
    this->robot_state_.ddq_d[i] = 0.0;
    this->robot_state_.tau_J_d[i] = arm_joint_effort_desired_[i];

    // For now we assume no flexible joints
    this->robot_state_.theta[i] = arm_joint_position_[i];
    this->robot_state_.dtheta[i] = arm_joint_velocity_[i];
    this->robot_state_.tau_ext_hat_filtered[i] = arm_joint_effort_[i];
  }

  this->robot_state_.control_command_success_rate = 1.0;
  this->robot_state_.time = franka::Duration(time.toNSec() / 1e6 /*ms*/);

  // TODO publish ros
  // TODO base pose
  // TODO base twist
  // TODO arm state
  // TODO object pose
  // TODO tank state
}

void RoyalPandaSim::publish_ros(){
  dynamics_->publish_ros();
}

void RoyalPandaSim::write_sim(ros::Time time, ros::Duration period){
  // TODO subscriber to the base desired twist
  u_.head<3>() = base_twist_desired_;
  dynamics_->set_control(u_);
  Eigen::VectorXd tau = 0.9 * dynamics_->get_panda()->getNonlinearities().e();
  tau.segment<7>(3) += arm_joint_effort_desired_;
  dynamics_->get_panda()->setGeneralizedForce(tau);
  dynamics_->advance();
}

double RoyalPandaSim::get_time_step() {
  return dynamics_->get_dt();
}