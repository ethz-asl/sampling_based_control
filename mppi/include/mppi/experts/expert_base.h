//
// Created by etienne on 14.12.20.
//

#pragma once

#include <Eigen/Dense>
#include <map>

#include "mppi/sampler/gaussian_sampler.h"
#include "mppi/solver_config.h"
#include "mppi/dynamics/dynamics_base.h"


class ExpertBase{
 public:
	using config_t = mppi::SolverConfig;
	using dynamics_ptr = mppi::DynamicsBase::dynamics_ptr;

  ExpertBase(char short_name, config_t config, dynamics_ptr dynamics){ short_name_ = short_name; config_ = config; dynamics_ = dynamics;};
  ~ExpertBase() = default;

  virtual void update_expert(std::vector<Eigen::VectorXd> mean)=0;
  virtual Eigen::VectorXd get_sample(size_t step)=0;

	virtual	Eigen::MatrixXd get_sigma_inv(size_t step)=0;
  char short_name_;

protected:
	config_t config_;
	dynamics_ptr dynamics_;

};
