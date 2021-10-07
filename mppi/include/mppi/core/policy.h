//
// Created by giuseppe on 31.05.21.
//

#pragma once
#include <vector>
#include <Eigen/Core>

namespace mppi {

/// Generic policy class for implementing a policy
/// Here the policy is intended as stochastic, therefore this class implements
/// also the methods to generate samples out of a nominal policy. These can be
/// used by the controller to generate rollouts
class Policy {
   public:
    Policy(int nu) : nu_(nu){}

    /**
     * Update the samples using "performance" weights assigned to each of them.
     * The parameter keep can be used to tell the policy to keep the best n out
     * of all the samples (according to the associated weights)
     */
    virtual void update_samples(const std::vector<double>& weights, const int keep) = 0;

    /**
     * Return the nominal policy
     */
    virtual Eigen::VectorXd nominal(double t) = 0;

    /**
     * Return a single sample
     */
    virtual Eigen::VectorXd sample(double t, int k) = 0;

    /**
     * Update the policy, given a set of sample weights and a step size for the
     * gradient step
     */
    virtual void update(const std::vector<double>& weights, const double step_size) = 0;

    /**
     * Shift the policy to the new time t
     */
    virtual void shift(const double t) = 0;

    /**
     * Bound the input to the predefined input bounds
     */
    virtual void bound() = 0;

    /**
     * Used to change policy behavior according to the "real time" number of
     * delay steps in computing the solution. E.g if the solver takes n steps
     * to solve the new optimization (sampling + optimization), the policy can
     * update only samples n steps in the future as they will be the ones
     * eventually used
     */
    virtual void update_delay(const int delay_steps){};

    /**
     * Set externally the nominal input trajectory
     */
    virtual void set_nominal(const Eigen::MatrixXd& nominal){};

   protected:
    int nu_;
  };
}


