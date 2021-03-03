/*!
 * @file     controller_ros.h
 * @author   Giuseppe Rizzi
 * @date     08.09.2020
 * @version  1.0
 * @brief    description
 */
#pragma once
#include <mppi/controller/mppi.h>

#include <ros/node_handle.h>
#include <mppi_ros/threading/WorkerManager.hpp>

#include <mppi_ros/Data.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>

using namespace mppi;
namespace mppi_ros {

class ControllerRos {
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
  virtual bool update_reference();
  virtual bool init_ros() { return true; };
  virtual void publish_ros(){};

  /**
   * @brief Init the ros pub/sub and set the controller object
   * @return true if initialization was successful
   */
  bool init();

  /**
   * @brief Starts all the threads
   * @return
   */
  bool start();

  /**
   * @brief Stop the running controller and associated threads
   */
  void stop();

  inline std::shared_ptr<PathIntegral>& get_controller() {
    return controller_;
  };
  const ros::NodeHandle& get_node_handle() { return nh_; }

  // TODO(giuseppe) this is dangerous since one might not use correctly this
  // class
  // TODO(giuseppe) split a sync vs an async class
  /**
   * Methods to use in a synchronous setup. In this case the call should follow
   * the pattern:
   * 1. set_observation: set last estimated state
   * 2. update_policy: optimize from just set observation
   * 3.
   *  a. get_input : only feedforward term required
   *  b. get_input_state: feedforward + nominal state
   * 4. (optional) publish_ros_default: run default publishing behavior
   * 5. (optional) publish_ros: run custom ros publishing
   */
  void set_observation(const observation_t& x, const double& t);
  bool update_policy();
  void get_input(const observation_t& x, input_t& u, const double& t);
  void get_input_state(const observation_t& x, observation_t& x_nom, input_t& u,
                       const double& t);
  bool publish_ros_default();

 private:
  void init_default_ros();
  bool init_default_params();

  bool update_policy_thread(const mppi::threading::WorkerEvent& event);
  bool update_reference_thread(const mppi::threading::WorkerEvent& event);
  bool publish_ros_thread(const mppi::threading::WorkerEvent& event);

  void publish_stage_cost();
  void publish_rollout_cost();
  void publish_input();

 private:
  bool started_;
  bool initialized_;
  bool observation_set_;

  mppi::threading::WorkerManager worker_manager_;

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

  // logging
  mppi_ros::Data data_ros_;
  ros::Publisher data_publisher_;
};

}  // namespace mppi_ros
