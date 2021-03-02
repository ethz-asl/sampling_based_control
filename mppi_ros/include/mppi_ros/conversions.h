//
// Created by giuseppe on 05.02.21.
//

#pragma once

#include <mppi/controller/data.h>
#include <mppi/controller/rollout.h>
#include <mppi/solver_config.h>

#include "mppi_ros/Array.h"
#include "mppi_ros/Config.h"
#include "mppi_ros/Data.h"
#include "mppi_ros/Rollout.h"

namespace mppi_ros {

void to_msg(const mppi::SolverConfig& config, Config& config_ros);
void to_msg(const mppi::Rollout& rollout, Rollout& rollout_ros);
void to_msg(const mppi::data_t& data, Data& data_ros);

}  // namespace mppi_ros