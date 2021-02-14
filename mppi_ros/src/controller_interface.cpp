/*!
 * @file     controller_interface.cpp
 * @author   Giuseppe Rizzi
 * @date     09.09.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_ros/controller_interface.h"

using namespace mppi_ros;

ControllerRos::ControllerRos(ros::NodeHandle &nh) : nh_(nh), observation_set_(false) {}

ControllerRos::~ControllerRos() { this->stop(); }

bool ControllerRos::init_default_params() {
  policy_update_rate_ = param_io::param(nh_, "policy_update_rate", 0.0);
  reference_update_rate_ = param_io::param(nh_, "reference_update_rate", 0.0);
  publish_ros_ = param_io::param(nh_, "publish_ros", false);
  ros_publish_rate_ = param_io::param(nh_, "ros_publish_rate", 0.0);
  return true;
}

void ControllerRos::init_default_ros() {
  cost_publisher_ = nh_.advertise<std_msgs::Float64>("/cost", 10);
  input_publisher_ = nh_.advertise<std_msgs::Float32MultiArray>("/input", 10);
  variance_publisher_ = nh_.advertise<std_msgs::Float32MultiArray>("/variance", 10);
  min_rollout_cost_publisher_ =
      nh_.advertise<std_msgs::Float64>("/min_rollout_cost", 10);
  max_rollout_cost_publisher_ =
      nh_.advertise<std_msgs::Float64>("/max_rollout_cost", 10);
}

bool ControllerRos::init() {
  init_default_params();
  init_default_ros();
  init_ros();

  bool ok;
  try{
    ok = set_controller(controller_);
  } catch (std::runtime_error& err){
    ROS_ERROR_STREAM(err.what());
    ok = false;
  } catch (...){
    ROS_ERROR("Unknown exception caught while setting the controller.");
    ok = false;
  }
  if (controller_ == nullptr || !ok) return false;
  filter_ = SavGolFilter(controller_->steps_, controller_->config_.filters_window.size(), controller_->config_.filters_window,
               controller_->config_.filters_order);
  initialized_ = true;
  started_ = false;
  ROS_INFO("Controller interface initialized.");
  return initialized_;
}

bool ControllerRos::start() {
  if (!initialized_) {
    ROS_ERROR_STREAM(
        "The controller is not initialized. Have you called the init() "
        "method?");
    return false;
  }

  if (started_){
    ROS_WARN_STREAM("The controller has already been started.");
    return true;
  }

  any_worker::WorkerOptions update_policy_opt;
  update_policy_opt.name_ = "update_policy_thread";
  update_policy_opt.timeStep_ =
      (policy_update_rate_ == 0) ? 0 : 1.0 / policy_update_rate_;
  update_policy_opt.callback_ = std::bind(&ControllerRos::update_policy_thread,
                                          this, std::placeholders::_1);
  worker_manager_.addWorker(update_policy_opt, true);

  any_worker::WorkerOptions update_reference_opt;
  update_reference_opt.name_ = "update_reference_thread";
  update_reference_opt.timeStep_ = 1.0 / reference_update_rate_;
  update_reference_opt.callback_ = std::bind(
      &ControllerRos::update_reference_thread, this, std::placeholders::_1);
  worker_manager_.addWorker(update_reference_opt, true);

  if (publish_ros_) {
    any_worker::WorkerOptions publish_ros_opt;
    publish_ros_opt.name_ = "publish_ros_thread";
    publish_ros_opt.timeStep_ = 1.0 / ros_publish_rate_;
    publish_ros_opt.callback_ = std::bind(&ControllerRos::publish_ros_thread,
                                          this, std::placeholders::_1);
    worker_manager_.addWorker(publish_ros_opt, true);
  }

  started_ = true;
  return true;
}

bool ControllerRos::update_policy() {
  if (!observation_set_) return true;
  controller_->update_policy(); 
  return true;
}

bool ControllerRos::update_policy_thread(const any_worker::WorkerEvent &event) {
  if (!observation_set_) return true;
  controller_->update_policy();
  return true;
}

bool ControllerRos::update_reference() { return true; }

bool ControllerRos::update_reference_thread(
    const any_worker::WorkerEvent &event) {
  return update_reference();
}

bool ControllerRos::publish_ros_default() {
  publish_input();
  publish_stage_cost();
  publish_rollout_cost();
  return true;
};

bool ControllerRos::publish_ros_thread(const any_worker::WorkerEvent &event) {
  publish_ros_default();
  publish_ros();
  return true;
}

void ControllerRos::publish_stage_cost() {
  stage_cost_.data = 0.0;  // TODO(giuseppe) see how to do this
  cost_publisher_.publish(stage_cost_);
}

void ControllerRos::publish_rollout_cost() {
  min_rollout_cost_.data = controller_->get_rollout_min_cost();
  min_rollout_cost_publisher_.publish(min_rollout_cost_);

  max_rollout_cost_.data = controller_->get_rollout_max_cost();
  max_rollout_cost_publisher_.publish(max_rollout_cost_);
}

void ControllerRos::publish_input() {
  mppi::input_t input_copy_;
  {
    std::shared_lock<std::shared_mutex> lock_input(input_mutex_);
    input_copy_ = input_;
  }

  input_ros_.data.resize(input_copy_.size());
  for (size_t i = 0; i < input_copy_.size(); i++) {
    input_ros_.data[i] = input_copy_(i);
  }
  input_publisher_.publish(input_ros_);

  Eigen::VectorXd var(input_copy_.size());
  controller_->get_diagonal_variance(var);
  var_ros_.data.resize(var.size());
  for (size_t i = 0; i < var.size(); i++) {
    var_ros_.data[i] = var(i);
  }
  variance_publisher_.publish(var_ros_);
}

void ControllerRos::set_observation(const mppi::observation_t &x,
                                    const mppi::time_t &t) {
  controller_->set_observation(x, t);
  observation_set_ = true;
}

void ControllerRos::get_input(const mppi::observation_t &x, mppi::input_t &u,
                              const mppi::time_t &t) {
  controller_->get_input(x, u, t);
//  filter_.reset(t);
//  filter_.add_measurement(u, t);
//  filter_.apply(u, t);

  if (publish_ros_) {
    std::unique_lock<std::shared_mutex> lock_input(input_mutex_);
    input_ = u;
  }
}

void ControllerRos::get_input_state(const observation_t &x,
                                    observation_t &x_nom, input_t &u,
                                    const mppi::time_t &t) {
  controller_->get_input_state(x, x_nom, u, t);
//  filter_.reset(t);
//  filter_.add_measurement(u, t);
//  filter_.apply(u, t);

  if (publish_ros_) {
    std::unique_lock<std::shared_mutex> lock_input(input_mutex_);
    input_ = u;
  }
}

void ControllerRos::stop() {
  worker_manager_.stopWorkers();
}
