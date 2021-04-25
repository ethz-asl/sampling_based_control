/*!
 * @file     learned_expert.h
 * @author   Andreas Voigt
 * @date     23.03.2021
 * @version  1.0
 * @brief    Interface for informing the trajectory sampling in MPPI by policy learning.
 */

#pragma once

#include <Eigen/Core>
#include <memory>
#include "mppi/controller/rollout.h"

namespace mppi {

class LearnedExpert {
  public:
    using learned_expert_ptr = std::shared_ptr<LearnedExpert>;
    using observation_t = Eigen::VectorXd;
    using input_t = Eigen::VectorXd;

    LearnedExpert(size_t state_dim, size_t input_dim);
    virtual ~LearnedExpert(){};

  public:
    
    /**
   * @brief Query the learned policy at a state to get an action
   * @details This function must be thread safe as it is potentially called 
   * simultaneously from multiple threads
   * @param x: state of interest
   * @return action returned from the learned policy
   */
    virtual input_t const get_action(const observation_t& x) = 0;

    /**
   * @brief Add the state action pair to the data set used for learning
   * @param x: observed state
   * @param u: planned action
   */
    virtual void save_state_action(const observation_t& x, const input_t& u) = 0;

    /**
   * @brief Add rollout to data set by repeatedly calling save_state_action 
   * @param xx: planned state trajectory
   * @param uu: planned actions
   */
    void save_rollout(const Rollout& rollout);

  protected:
    size_t state_dim_;
    size_t input_dim_;
};

} // namespace mppi
