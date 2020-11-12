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

#include <math.h>
#include <iostream>
#include <stdexcept>
#include <Eigen/Core>
#include <mppi/dynamics/dynamics_base.h>
#include <ros/package.h>
#include <numeric>
#include "mppi_panda_raisim/dimensions.h"

namespace panda{

class PandaRaisimDynamics : public mppi::DynamicsBase {
 public:
  PandaRaisimDynamics(const std::string& robot_description, const double dt);

  ~PandaRaisimDynamics() = default;

 private:
  void initialize_world(const std::string robot_description, const double dt);
  void initialize_pd();
  void set_collision();


 public:
  size_t get_input_dimension() override { return PandaDim::INPUT_DIMENSION; }
  size_t get_state_dimension() override { return PandaDim::STATE_DIMENSION; }
  dynamics_ptr create() override { return std::make_shared<PandaRaisimDynamics>(robot_description_, dt_); }

  dynamics_ptr clone() const override{
    std::cout << "cannot clone, raisim world copy constructor is deleted. Returning empty pointer" << std::endl;
    return dynamics_ptr();
  }

  void reset(const observation_t &x) override;

  observation_t step(const input_t &u, const double dt) override;
  input_t get_zero_input(const observation_t& x) override;

 private:
  observation_t x_;

  double dt_;
  std::string robot_description_;
  raisim::ArticulatedSystem* panda;
  raisim::ArticulatedSystem* door;

  raisim::World sim_;

  Eigen::VectorXd door_p, door_v;
  Eigen::VectorXd cmd, cmdv;
  Eigen::VectorXd joint_p, joint_v;
  Eigen::VectorXd joint_p_gain, joint_d_gain;
  Eigen::VectorXd joint_p_desired, joint_v_desired;

};
}
