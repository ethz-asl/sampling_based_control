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

namespace mobile_panda{

enum PandaDim{
  STATE_DIMENSION = 10, // 7 joints plus x, y, yaw
  INPUT_DIMENSION = 9   // 7 joints plus v and yaw_dot
};

struct PandaDynamicsConfig{
  double substeps = 1;
};

class PandaDynamics : public mppi::DynamicsBase {
 public:
  PandaDynamics(){

    x_ = observation_t::Zero(PandaDim::STATE_DIMENSION);

    // initialize dynamics
    std::string urdf_path = ros::package::getPath("mppi_mobile_panda");
    urdf_path += "/resources/panda/panda.urdf";
    std::cout << "Parsing model from: " << urdf_path << std::endl;
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
  };
  ~PandaDynamics() = default;

 public:
  void set_dynamic_properties(const PandaDynamicsConfig& config){ config_ = config;}

  size_t get_input_dimension() override { return PandaDim::INPUT_DIMENSION; }
  size_t get_state_dimension() override { return PandaDim::STATE_DIMENSION; }

  dynamics_ptr create() override {
    return std::make_shared<PandaDynamics>();
  }

  dynamics_ptr clone() const override {
    return std::make_shared<PandaDynamics>(*this);
  }

  void reset(const observation_t &x) override;

  observation_t step(const input_t &u, const double dt) override;
  input_t get_zero_input(const observation_t& x) override;

 private:
  observation_t x_;
  PandaDynamicsConfig config_;

  pinocchio::Model model_;
  pinocchio::Data data_;

};
}


