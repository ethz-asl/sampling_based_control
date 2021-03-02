//
// Created by giuseppe on 05.02.21.
//

#include "mppi_ros/conversions.h"

namespace mppi_ros {

void to_msg(const mppi::SolverConfig& config, Config& config_ros) {
  config_ros.nr_rollouts = config.rollouts;
  config_ros.lambda = config.lambda;
  config_ros.h = config.h;
  config_ros.alpha = config.alpha;
  config_ros.beta = config.beta;
  config_ros.step_size = config.step_size;
  config_ros.caching_factor = config.caching_factor;
  config_ros.horizon = config.horizon;
  config_ros.adaptive_sampling = config.adaptive_sampling;
  config_ros.input_variance.array.assign(
      config.input_variance.data(),
      config.input_variance.data() + config.input_variance.size());
  config_ros.filtering = config.filtering;
  config_ros.cost_ratio = config.cost_ratio;
  config_ros.discount_factor = config.discount_factor;
  config_ros.verbose = config.verbose;
  config_ros.debug_print = config.debug_print;
  config_ros.bound_input = config.bound_input;
  config_ros.input_min.array.assign(config.u_min.data(),
                                    config.u_min.data() + config.u_min.size());
  config_ros.input_max.array.assign(config.u_max.data(),
                                    config.u_max.data() + config.u_max.size());
  config_ros.filter_order = config.filter_order;
  config_ros.filter_window = config.filter_window;
  config_ros.tree_search = config.use_tree_search;
}

void to_msg(const mppi::Rollout& rollout, Rollout& rollout_ros) {
  rollout_ros.steps = rollout.steps_;
  rollout_ros.input_dim = rollout.input_dim_;
  rollout_ros.state_dim = rollout.state_dim_;
  rollout_ros.total_cost = rollout.total_cost;

  rollout_ros.cost_vector.array.assign(rollout.cc.data(),
                                       rollout.cc.data() + rollout.cc.size());
  rollout_ros.time_vector.array.assign(rollout.tt.begin(), rollout.tt.end());

  mppi_ros::Array state, input, noise;
  for (size_t i = 0; i < rollout.steps_; i++) {
    state.array.assign(rollout.xx[i].data(),
                       rollout.xx[i].data() + rollout.xx[i].size());
    input.array.assign(rollout.uu[i].data(),
                       rollout.uu[i].data() + rollout.uu[i].size());
    noise.array.assign(rollout.nn[i].data(),
                       rollout.nn[i].data() + rollout.nn[i].size());
    rollout_ros.state_vector.push_back(state);
    rollout_ros.input_vector.push_back(input);
    rollout_ros.noise_vector.push_back(noise);
  }
}

void to_msg(const mppi::data_t& data, Data& data_ros) {
  data_ros.time = ros::Time::now();
  to_msg(data.config, data_ros.config);

  if (!data.weights.size()) return;

  // to_msg(data.optimal_rollout, data_ros.optimal_rollout);

  //  mppi_ros::Rollout rollout_ros;
  //  for (const auto& roll : data.rollouts){
  //    to_msg(roll, rollout_ros);
  //    data_ros.rollouts.push_back(rollout_ros);
  //  }
  data_ros.step_count = data.step_count;
  data_ros.stage_cost = data.stage_cost;
  data_ros.reset_time = data.reset_time;
  data_ros.optimization_time = data.optimization_time;
  data_ros.rollouts_cost.array.assign(
      data.rollouts_cost.data(),
      data.rollouts_cost.data() + data.rollouts_cost.size());
  data_ros.weights.array.assign(data.weights.data(),
                                data.weights.data() + data.weights.size());
}

}  // namespace mppi_ros