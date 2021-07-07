/*!
 * @file     controller_ros.cpp
 * @author   Giuseppe Rizzi
 * @date     03.09.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_pole_cart/controller_interface.h"
#include "policy_learning/pole_cart_expert.h"
#include "policy_learning/hdf5_dataset.h"
#include "policy_learning/torch_script_policy.h"

#include <ros/package.h>

using namespace pole_cart;

bool PoleCartControllerInterface::set_controller(
    std::shared_ptr<mppi::PathIntegral> &controller) {
  // -------------------------------
  // dynamics
  // -------------------------------
  auto dynamics_config = PoleCartDynamicsConfig();
  dynamics_config.mc = nh_.param<double>("dynamics/mass_cart", 1.0);
  dynamics_config.mp = nh_.param<double>("dynamics/mass_pendulum", 0.5);
  dynamics_config.l = nh_.param<double>("dynamics/length", 1.0);
  dynamics_config.mux = nh_.param<double>("dynamics/linear_friction", 10.0);
  dynamics_config.mutheta = nh_.param<double>("dynamics/angular_friction", 0.7);
  dynamics_config.dt_internal =
      nh_.param<double>("dynamics/substep_size", 0.001);

  auto dynamics = std::make_shared<PoleCartDynamics>(dynamics_config);

  // -------------------------------
  // cost
  // -------------------------------
  auto cost = std::make_shared<PoleCartCost>();

  // -------------------------------
  // config
  // -------------------------------
  std::string config_file =
      ros::package::getPath("mppi_pole_cart") + "/config/params.yaml";
  if (!config_.init_from_file(config_file)) {
    ROS_ERROR_STREAM("Failed to init solver options from " << config_file);
    return false;
  }

  // -------------------------------
  // learner
  // -------------------------------
  std::unique_ptr<Policy> policy_ptr = nullptr;
  std::string torchscript_model_path;
  if (nh_.param<std::string>("torchscript_model_path", 
      torchscript_model_path, "")) {
    policy_ptr = std::make_unique<TorchScriptPolicy>(
      torchscript_model_path
    );
    ROS_INFO_STREAM("Using Torch script from " << torchscript_model_path);
  }
  std::unique_ptr<Dataset> dataset_ptr = nullptr;
  std::string learned_expert_output_path;
  if (nh_.param<std::string>("learned_expert_output_path", 
      learned_expert_output_path, "")) {
    dataset_ptr = std::make_unique<Hdf5Dataset>(
      learned_expert_output_path
    );
    ROS_INFO_STREAM("HDF5 output path: " << learned_expert_output_path);
  }

  auto learner = std::make_shared<PoleCartExpert>(
    std::move(policy_ptr), 
    std::move(dataset_ptr)
  );  

  // -------------------------------
  // controller
  // -------------------------------
  controller = std::make_shared<mppi::PathIntegral>(
    dynamics, cost, config_, nullptr, nullptr, learner);

  // -------------------------------
  // initialize reference
  // -------------------------------
  // cart at the origin, pole up
  ref_.rr.resize(1,
                 mppi::observation_t::Zero(PoleCartDim::REFERENCE_DIMENSION));
  ref_.rr[0](0) = 0.0;
  ref_.rr[0](1) = M_PI;
  ref_.tt.resize(1, 0.0);
  return true;
}

bool PoleCartControllerInterface::update_reference() {
  if (!reference_set_) get_controller()->set_reference_trajectory(ref_);
  reference_set_ = true;
  return true;
}

void PoleCartControllerInterface::publish_ros() {}