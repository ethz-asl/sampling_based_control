//
// Created by giuseppe on 05.02.21.
//

#pragma once

#include <Eigen/Core>

#include "mppi/controller/rollout.h"
#include "mppi/solver_config.h"

namespace mppi {

struct data_t {
  SolverConfig config;

  double step_count;
  double stage_cost;
  double reset_time;
  double optimization_time;

  Eigen::ArrayXd rollouts_cost;
  Eigen::ArrayXd weights;
  std::vector<Rollout> rollouts;
  Rollout optimal_rollout;
};

}  // namespace mppi
