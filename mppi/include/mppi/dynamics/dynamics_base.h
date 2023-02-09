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

namespace mppi {
class DynamicsBase {
 public:
  using dynamics_ptr = std::shared_ptr<DynamicsBase>;
  using observation_t = Eigen::VectorXd;
  using input_t = Eigen::VectorXd;
  using input_array_t = Eigen::VectorXd;
  DynamicsBase() = default;
  virtual ~DynamicsBase() = default;

 public:
  virtual size_t get_input_dimension() = 0;
  virtual size_t get_state_dimension() = 0;

  virtual dynamics_ptr create() = 0;       // virtual constructor
  virtual dynamics_ptr clone() const = 0;  // virtual copy constructor

  virtual void reset(const observation_t& x) = 0;
  virtual void render() {}
  virtual observation_t step(const input_t& u, const double dt) = 0;
  virtual const observation_t get_state() const = 0;

  /**
   * @brief Convert an actual observed state to the extended state vector used
   * in the implementation specific dynamics simulation. Needs to be implemented
   * accordingly in the derived dynamics class if these two vectors differ.
   * @param x: current actual observation
   * @return extended state vector used in the dynamics simulation
   */
  virtual observation_t get_extended_state_from_observation(
      const observation_t& x) const {
    return x;
  }

  // TODO(giuseppe): param x is never used!
  virtual input_t get_zero_input(const observation_t& x) {
    return input_t::Zero(this->get_input_dimension());
  }
};

}  // end of namespace mppi
