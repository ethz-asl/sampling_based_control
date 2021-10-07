/*!
 * @file     dynamics_base.h
 * @author   Giuseppe Rizzi
 * @date     01.07.2020
 * @version  1.0
 * @brief    description
 */

#pragma once

#include <Eigen/Core>
#include <memory>
#include "mppi/core/typedefs.h"

namespace mppi {
class Dynamics {
 public:
  Dynamics() = default;
  ~Dynamics() = default;

 public:
  virtual size_t get_input_dimension() = 0;
  virtual size_t get_state_dimension() = 0;

  virtual dynamics_ptr create() = 0;       // virtual constructor
  virtual dynamics_ptr clone() const = 0;  // virtual copy constructor

  /**
   * Reset the current state of the world to x at the time t
   */
  virtual void reset(const observation_t& x, const double t) = 0;

  /**
   * Given the current input and simulation dt, predicts the state at
   * the next time step. Notice that dt does not need to be used,
   * as the dynamics function might have its own internal dt fixed at
   * creation time (e.g when using simulators)
   */
  virtual observation_t step(const input_t& u, const double dt) = 0;

  /**
   * Return the current state of the dynamics
   */
  virtual const observation_t get_state() const = 0;

  /**
   * Used to extend the input sequence when extening the horizon
   */
  virtual input_t get_zero_input(const observation_t& x) {
    return input_t::Zero(this->get_input_dimension());
  }

  inline double get_dt() const { return dt; }

 protected:
  double t_;
  double dt;
};

}  // end of namespace mppi
