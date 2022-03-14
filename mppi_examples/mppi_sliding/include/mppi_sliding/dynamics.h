/*!
 * @file     dynamics_raisim.h
 * @author   Giuseppe Rizzi
 * @date     05.10.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#include <raisim/World.hpp>
#include <raisim/configure.hpp>
#include <raisim/RaisimServer.hpp>

#include <mppi/core/dynamics.h>
#include <ros/package.h>
#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include "mppi_sliding/dimensions.h"
#include "mppi_sliding/params/dynamics_params.h"

namespace manipulation {

struct force_t {
  Eigen::Vector3d force;
  Eigen::Vector3d position;
};

class PandaRaisimDynamics : public mppi::Dynamics {
 public:
  PandaRaisimDynamics(const DynamicsParams& params, const bool if_sim);  //if_sim is true means is to simulate the "real world dynamics"
  ~PandaRaisimDynamics() = default;
 private:
  void initialize_world(const std::string& robot_description,
                        const std::string& object_description);
                        
  void initialize_pd();
  void set_collision();
  void display_state();
  void update_geometry();

 protected:
  const bool if_sim_;
  bool if_update_;
  Eigen::VectorXd state_buffer;

  const std::vector<raisim::Vec<2>> object_limits_;

 public:
  double get_dt() { return dt_; }
  
  size_t get_input_dimension() override { return input_dimension_; }
  
  size_t get_state_dimension() override { return state_dimension_; }
  
  mppi::dynamics_ptr create() override {
    return std::make_shared<PandaRaisimDynamics>(params_,if_sim_);
  }

  mppi::dynamics_ptr clone() const override {
    std::cout << "cannot clone, raisim world copy constructor is deleted. "
                 "Returning empty pointer"
              << std::endl;
    return mppi::dynamics_ptr();
  }

  void reset(const mppi::observation_t& x, const double t) override;
  
  void advance();
  
  void set_control(const mppi::input_t& u);

  mppi::observation_t step(const mppi::input_t& u, const double dt) override;
  
  mppi::input_t get_zero_input(const mppi::observation_t& x) override;
  
  const mppi::observation_t get_state() const override { return x_; }

  Eigen::VectorXd get_primitive_state();

  raisim::World* get_world() { return &sim_; }
  
  raisim::ArticulatedSystem* get_panda() { return panda_; }
  
  raisim::ArticulatedSystem* get_object() { return object_; }
  
  std::vector<force_t> get_contact_forces();
  
  void get_end_effector_pose(Eigen::Vector3d& position,
                             Eigen::Quaterniond& orientation);

  void get_handle_pose(Eigen::Vector3d& position,
                       Eigen::Quaterniond& orientation);

  // this is a link on royalpanda which is tracked by vicon
  void get_reference_link_pose(Eigen::Vector3d& position,
                               Eigen::Quaterniond& orientation);

  double get_object_displacement() const;
  
  void get_external_torque(Eigen::VectorXd& tau_ext);
  
  void get_external_wrench(Eigen::VectorXd& wrench);
  
  void get_ee_jacobian(Eigen::MatrixXd& J);
  
  void set_external_ee_force(const Eigen::Vector3d& f);
  
  void fix_object();
  
  void release_object();

 protected:
  size_t robot_dof_, obj_dof_;
  size_t input_dimension_;
  size_t state_dimension_;

  mppi::observation_t x_;
  Eigen::VectorXd tau_ext_;
  Eigen::VectorXd joint_p_, joint_v_;

 protected:

  double dt_;
  raisim::Vec<3> gravity_;
  DynamicsParams params_;
  
  std::string robot_description_;
  std::string mug_description_;

  raisim::World sim_;

  raisim::ArticulatedSystem* panda_;
  raisim::ArticulatedSystem* object_;
  raisim::ArticulatedSystem* mug_;
  raisim::Box* table_;
  raisim::Cylinder* cylinder_;
  raisim::Cylinder* cylinder_2;


  Eigen::VectorXd object_p_, object_v_;
  Eigen::VectorXd box_p_, box_v_;
  Eigen::VectorXd cmd_, cmdv_;
  Eigen::VectorXd joint_p_gain_, joint_d_gain_;
  Eigen::VectorXd joint_p_desired_, joint_v_desired_;

  bool ee_force_applied_;
  Eigen::MatrixXd J_contact_;
  Eigen::VectorXd torque_ext_;
};
}  // namespace manipulation
