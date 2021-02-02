/*!
 * @file     pendulumcartdynamics.h
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#pragma once
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/aba.hpp>

#include <math.h>
#include <iostream>
#include <stdexcept>
#include <Eigen/Core>
#include <mppi/dynamics/dynamics_base.h>
#include <ros/package.h>

#include "mppi_panda/dimensions.h"

namespace panda{

struct PandaDynamicsConfig{
  double substeps = 1;
};

class PandaDynamics : public mppi::DynamicsBase {
 public:
  PandaDynamics(const std::string& robot_description=""): robot_description_(robot_description){

    x_ = observation_t::Zero(PandaDim::STATE_DIMENSION);
    previous_u_ = input_t::Zero(PandaDim::INPUT_DIMENSION);

    // initialize model
    pinocchio::urdf::buildModelFromXML(robot_description, model_);
    data_ = pinocchio::Data(model_);
  };
  ~PandaDynamics() = default;

 public:
  void set_dynamic_properties(const PandaDynamicsConfig& config){ config_ = config;}

  size_t get_input_dimension() override { return PandaDim::INPUT_DIMENSION; }
  size_t get_state_dimension() override { return PandaDim::STATE_DIMENSION; }

  dynamics_ptr create() override {
    return std::make_shared<PandaDynamics>(robot_description_);
  }

  dynamics_ptr clone() const override {
    return std::make_shared<PandaDynamics>(*this);
  }

  void reset(const observation_t &x) override;

  observation_t step(const input_t &u, const double dt) override;
  const observation_t get_state() const override;
  input_t get_zero_input(const observation_t& x) override;

 private:
  observation_t x_;
  observation_t xdd_;
  input_t previous_u_;
  PandaDynamicsConfig config_;

  std::string robot_description_;
  pinocchio::Model model_;
  pinocchio::Data data_;

};
}


