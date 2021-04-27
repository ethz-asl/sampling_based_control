//
// Created by giuseppe on 05.02.21.
//

#pragma once

#include <rclcpp/time.hpp>

#include <mppi/controller/data.h>
#include <mppi/controller/rollout.h>
#include <mppi/solver_config.h>

#include "mppi_ros/msg/array.hpp"
#include "mppi_ros/msg/config.hpp"
#include "mppi_ros/msg/data.hpp"
#include "mppi_ros/msg/rollout.hpp"

namespace mppi_ros {

void to_msg(const mppi::SolverConfig& config, msg::Config& config_ros);
void to_msg(const mppi::Rollout& rollout, msg::Rollout& rollout_ros);
void to_msg(const mppi::data_t& data, msg::Data& data_ros, const rclcpp::Time& time);

}  // namespace mppi_ros