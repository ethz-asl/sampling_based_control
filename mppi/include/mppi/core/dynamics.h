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

  virtual void reset(const observation_t& x, const double t) = 0;
  virtual void render() {}
  virtual observation_t step(const input_t& u, const double dt) = 0;
  virtual const observation_t get_state() const = 0;

  // TODO(giuseppe): param x is never used!
  virtual input_t get_zero_input(const observation_t& x) {
    return input_t::Zero(this->get_input_dimension());
  }

  inline double get_dt() const { return dt; }

 protected:
  double t_;
  double dt;
};

}  // end of namespace mppi
