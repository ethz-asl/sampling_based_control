/*!
 * @file     dynamics.h
 * @author   Matthias Studiger
 * @date     14.03.2021
 * @version  1.0
 * @brief    description
 */

#pragma once

#include <math.h>
#include <mppi/dynamics/dynamics_base.h>
#include <Eigen/Core>
#include <iostream>
#include <stdexcept>

namespace omav_pathfinder {

enum OMAV_PathfinderDim {
  STATE_DIMENSION = 19,     // B_f(3), B_tau(3), I_v(3), q(4), omega(3), position(3)
  INPUT_DIMENSION = 6,      // B_f_dot(3), B_tau_dot(3)
  REFERENCE_DIMENSION = 3   // position
};

struct OMAV_PathfinderDynamicsConfig {
  //TODO: Put updated values into it, since data is used at multiple locations,
  // add from parameter file.
  double mass = 4.04;
  double gravity = 9.8086;
  double Ix =  0.078359;
  double Iy =  0.081797;
  double Iz =  0.153355;
  double dt_internal = 0.001;
};

class OMAV_PathfinderDynamics : public mppi::DynamicsBase {
 public:
  OMAV_PathfinderDynamics() {
    x_ = observation_t::Zero(OMAV_PathfinderDim::STATE_DIMENSION);
    xd_ = observation_t::Zero(OMAV_PathfinderDim::STATE_DIMENSION);
  };

  explicit OMAV_PathfinderDynamics(const OMAV_PathfinderDynamicsConfig& config)
      : OMAV_PathfinderDynamics() {
    config_ = config;
  };

  ~OMAV_PathfinderDynamics() = default;

 public:
  void set_dynamic_properties(const OMAV_PathfinderDynamicsConfig& config) {
    config_ = config;
  }

  size_t get_input_dimension() override { return OMAV_PathfinderDim::INPUT_DIMENSION; }
  size_t get_state_dimension() override { return OMAV_PathfinderDim::STATE_DIMENSION; }

  dynamics_ptr create() override {
    return std::make_shared<OMAV_PathfinderDynamics>();
  }

  dynamics_ptr clone() const override {
    return std::make_shared<OMAV_PathfinderDynamics>(*this);
  }

  void reset(const observation_t& x) override;

  observation_t step(const input_t& u, const double dt) override;

  const observation_t get_state() const override;

 private:
    //TODO: Inputs more elegant (as arrays)
    void compute_velocities(double B_f_dot_x, double B_f_dot_y, double B_f_dot_z, double B_tau_dot_x, double B_tau_dot_y, double B_tau_dot_z);
    void integrate_internal(double B_f_dot_x, double B_f_dot_y, double B_f_dot_z, double B_tau_dot_x, double B_tau_dot_y, double B_tau_dot_z, double dt);

 private:
  OMAV_PathfinderDynamicsConfig config_;
  observation_t x_;
  observation_t xd_;
};
}  // namespace omav_pathfinder
