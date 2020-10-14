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
  PandaRaisimDynamics(const std::string& robot_description, const double dt){
    joint_p.setZero(PandaDim::JOINT_DIMENSION);
    joint_v.setZero(PandaDim::JOINT_DIMENSION);
    joint_p_gain.setZero(PandaDim::JOINT_DIMENSION);
    joint_d_gain.setZero(PandaDim::JOINT_DIMENSION);
    joint_p_desired.setZero(PandaDim::JOINT_DIMENSION);
    joint_v_desired.setZero(PandaDim::JOINT_DIMENSION);
    joint_p_gain.setConstant(200);
    joint_d_gain.setConstant(10.0);

    x_ = observation_t::Zero(PandaDim::STATE_DIMENSION);

    sim_.setTimeStep(dt);
    sim_.setERP(0.,0.);
    dt_ = dt;
    robot_description_ = robot_description;
    panda = sim_.addArticulatedSystem(robot_description_, "/");
    panda->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    panda->setPdGains(joint_p_gain, joint_d_gain);
    panda->setGeneralizedForce(Eigen::VectorXd::Zero(panda->getDOF()));

  };

  ~PandaRaisimDynamics() {
    std::cout << "\nAccurate average sim time: " << std::accumulate(time_recordings_.begin(), time_recordings_.end(), 0.0)/time_recordings_.size() << " ms. " << std::endl;
  };

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

  raisim::World sim_;

  Eigen::VectorXd joint_p, joint_v;
  Eigen::VectorXd joint_p_gain, joint_d_gain;
  Eigen::VectorXd joint_p_desired, joint_v_desired;

  std::vector<double> time_recordings_;

};
}
