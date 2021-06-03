//
// Created by giuseppe on 31.05.21.
//

#pragma once
#include <Eigen/Core>

namespace mppi {

class Policy {
   public:
    Policy(int nu) : nu_(nu){}

    virtual void update_samples(const Eigen::VectorXd& weights, const int keep) = 0;

    virtual Eigen::VectorXd operator()(double t) = 0;

    virtual Eigen::VectorXd operator()(double t, int k) = 0;

    virtual void update(const Eigen::VectorXd& weights, const double step_size) = 0;

    virtual void shift(const double t) = 0;

    virtual void bound() = 0;

   protected:
    int nu_;
  };
}


