/*!
 * @file     cost_base.h
 * @author   Giuseppe Rizzi
 * @date     01.07.2020
 * @version  1.0
 * @brief    description
 */

#pragma once
#include <Eigen/Core>
#include <memory>
#include <shared_mutex>
#include "mppi/core/typedefs.h"

namespace mppi {

class Cost {
 public:
  Cost() = default;
  ~Cost() = default;

 public:
  virtual cost_ptr create() = 0;       // virtual constructor
  virtual cost_ptr clone() const = 0;  // virtual copy constructor

  /**
   * @brief Compute the current stage cost
   * @param x: stage observation
   * @param time: stage time
   * @return the stage cost
   */
  virtual cost_t get_stage_cost(const observation_t& x, const input_t& u,
                                const double t = 0);

  /**
   * @brief Set the reference trajectory (optionally) used in the cost function
   * @param ref_traj the new timed reference trajectory
   */
  void set_reference_trajectory(const reference_trajectory_t& traj);

 private:
  /**
   * @brief The derived class must implement this in order to compute the cost
   * at the current time wrt to the given reference
   * @param x current observation
   * @param x_ref reference state extracted from reference trajectory
   * @param t current time
   * @return
   */
  virtual cost_t compute_cost(const observation_t& x, const input_t& u,
                              const reference_t& ref, const double t) = 0;

  /**
   * @brief Get the reference point closest to the current time.
   * @details Always the next point in time is considered in the default
   * implementation. Different types of interpolation could be implemented by
   * the derived class
   * @param[in]  x: current observation
   * @param[out] ref : the returned reference point
   * @param[in]  t : the query time
   */
  virtual void interpolate_reference(const observation_t& x, reference_t& ref,
                                     const double t);

 private:
  // single reference point in time
  reference_t r_;

  // timed sequence of references
  reference_trajectory_t timed_ref_;
};

}  // end of namespace mppi
