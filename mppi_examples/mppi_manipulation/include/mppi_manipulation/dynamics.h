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

#include <mppi/dynamics/dynamics_base.h>
#include <ros/package.h>
#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include "mppi_manipulation/dimensions.h"
#include "mppi_manipulation/gains.h"

namespace manipulation {

struct force_t {
  Eigen::Vector3d force;
  Eigen::Vector3d position;
};

class PandaRaisimDynamics : public mppi::DynamicsBase {
 public:
  PandaRaisimDynamics(const std::string& robot_description,
                      const std::string& object_description, const double dt,
                      const bool fixed_base = true,
                      const PandaRaisimGains& = PandaRaisimGains());
  ~PandaRaisimDynamics() = default;

 private:
  void initialize_world(const std::string& robot_description,
                        const std::string& object_description);
  void initialize_pd();
  void set_collision();

 public:
  double get_dt() { return dt_; }
  size_t get_input_dimension() override { return input_dimension_; }
  size_t get_state_dimension() override { return state_dimension_; }
  dynamics_ptr create() override {
    return std::make_shared<PandaRaisimDynamics>(
        robot_description_, object_description_, dt_, fixed_base_);
  }

  dynamics_ptr clone() const override {
    std::cout << "cannot clone, raisim world copy constructor is deleted. "
                 "Returning empty pointer"
              << std::endl;
    return dynamics_ptr();
  }

  void reset(const observation_t& x) override;

  observation_t step(const input_t& u, const double dt) override;
  input_t get_zero_input(const observation_t& x) override;
  const observation_t get_state() const override { return x_; }

  raisim::World* get_world() { return &sim_; }
  raisim::ArticulatedSystem* get_panda() { return panda; }
  raisim::ArticulatedSystem* get_object() { return object; }

  std::vector<force_t> get_contact_forces();
  void get_end_effector_pose(Eigen::Vector3d& position,
                             Eigen::Quaterniond& orientation);
  void get_handle_pose(Eigen::Vector3d& position,
                       Eigen::Quaterniond& orientation);
  double get_object_displacement() const;

 protected:
  bool fixed_base_;
  size_t robot_dof_;
  size_t input_dimension_;
  size_t state_dimension_;

  observation_t x_;

 private:
  double dt_;
  std::string robot_description_;
  std::string object_description_;

  raisim::ArticulatedSystem* panda;
  raisim::ArticulatedSystem* object;

  raisim::World sim_;

  Eigen::VectorXd object_p, object_v;
  Eigen::VectorXd cmd, cmdv;
  Eigen::VectorXd joint_p, joint_v;
  Eigen::VectorXd joint_p_gain, joint_d_gain;
  Eigen::VectorXd joint_p_desired, joint_v_desired;

  PandaRaisimGains gains_;
};
}  // namespace manipulation
