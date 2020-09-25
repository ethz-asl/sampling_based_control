/*!
 * @file     controller_ros.h
 * @author   Giuseppe Rizzi
 * @date     08.09.2020
 * @version  1.0
 * @brief    description
 */
#pragma once
#include <mppi/controller/mppi.h>

#include <any_worker/WorkerManager.hpp>
#include <param_io/get_param.hpp>
#include <ros/node_handle.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>

using namespace mppi;
namespace mppi_ros{

class ControllerRos{
 public:
  ControllerRos() = delete;
  ControllerRos(ros::NodeHandle& nh);
  ~ControllerRos();

  /**
   * @brief Must implement to set the sampling based controller
   * @param controller
   */
  virtual bool set_controller(std::shared_ptr<PathIntegral>& controller) = 0;

  /**
   * @brief Must implement to set the reference for the controller
   * @return
   */
  virtual bool update_reference() = 0;
  virtual bool init_ros(){};
  virtual void publish_ros(){};

  /**
   * @brief Set the controller and starts all the threads
   * @return
   */
  bool start();

  inline std::shared_ptr<PathIntegral>& get_controller() { return controller_; };
  const ros::NodeHandle& get_node_handle() {return nh_; }

  void set_observation(const observation_t& x, const mppi::time_t& t);
  void get_input(const observation_t& x, input_t& u, const mppi::time_t& t);
  void get_input_state(const observation_t& x, observation_t& x_nom, input_t& u, const mppi::time_t& t);

 private:
  void init_default_ros();
  bool init_default_params();

  bool update_policy(const any_worker::WorkerEvent& event);
  bool update_reference_l(const any_worker::WorkerEvent& event);

  void publish_stage_cost();
  void publish_rollout_cost();
  void publish_input();
  bool publish_ros_l(const any_worker::WorkerEvent& event);

 private:
  any_worker::WorkerManager worker_manager_;

  mppi::CostBase::cost_ptr cost_;
  mppi::DynamicsBase::dynamics_ptr dynamics_;
  mppi::SolverConfig config_;

  std::shared_ptr<mppi::PathIntegral> controller_ = nullptr;

  double policy_update_rate_ = 0.0;
  double reference_update_rate_ = 0.0;

  bool publish_ros_ = false;
  double ros_publish_rate_ = 0.0;

 public:
  ros::NodeHandle nh_;

  ros::Publisher cost_publisher_;
  ros::Publisher min_rollout_cost_publisher_;
  ros::Publisher max_rollout_cost_publisher_;
  ros::Publisher input_publisher_;

  std_msgs::Float64 stage_cost_;
  std_msgs::Float64 min_rollout_cost_;
  std_msgs::Float64 max_rollout_cost_;

  std::shared_mutex input_mutex_;
  mppi::input_t input_ = mppi::input_t::Zero(1);
  std_msgs::Float32MultiArray input_ros_;
};

}
