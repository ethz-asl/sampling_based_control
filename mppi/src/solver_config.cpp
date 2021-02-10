/*!
 * @file     solver_config.cpp
 * @author   Giuseppe Rizzi
 * @date     22.07.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi/solver_config.h"
#include <math.h>
#include <iostream>

using namespace mppi;

bool SolverConfig::init_from_file(const std::string &file) {
  parsing_error = false;
  YAML::Node config;
  try {
    config = YAML::LoadFile(file);
  } catch (const YAML::ParserException &ex) {
    std::cout << ex.what() << std::endl;
  } catch (const YAML::BadFile &ex) {
    std::cout << ex.what() << std::endl;
  }

  YAML::Node solver_options = config["solver"];
  if (!solver_options) {
    std::cout << "Failed to parse solver options." << std::endl;
    return false;
  }

  // clang-format off
  rollouts        = parse_key<size_t>(solver_options, "rollouts").value_or(rollouts);
  lambda          = parse_key<double>(solver_options, "lambda").value_or(lambda);
  h               = parse_key<double>(solver_options, "h").value_or(h);
  substeps        = parse_key_quiet<size_t>(solver_options, "substeps").value_or(substeps);
  caching_factor  = parse_key<double>(solver_options, "caching_factor").value_or(caching_factor);
  step_size       = parse_key<double>(solver_options, "step_size").value_or(step_size);
  horizon         = parse_key<double>(solver_options, "horizon").value_or(horizon);
  adaptive_sampling = parse_key_quiet<bool>(solver_options, "adaptive_sampling").value_or(adaptive_sampling);
  input_variance  = parse_key<Eigen::VectorXd>(solver_options, "input_variance").value_or(Eigen::VectorXd(0));
  filter_type     = (InputFilterType)parse_key_quiet<int>(solver_options, "filter_type").value_or(filter_type);
  filter_window   = parse_key_quiet<int>(solver_options, "filter_window").value_or(filter_window);
  filter_order    = parse_key_quiet<int>(solver_options, "filter_order").value_or(filter_order);
  filtering       = parse_key_quiet<bool>(solver_options, "filtering").value_or(filtering);
  bound_input     = parse_key_quiet<bool>(solver_options, "bound_input").value_or(false);
  u_min           = parse_key_quiet<Eigen::VectorXd>(solver_options, "u_min").value_or(Eigen::VectorXd(0));
  u_max           = parse_key_quiet<Eigen::VectorXd>(solver_options, "u_max").value_or(Eigen::VectorXd(0));
  cost_ratio      = parse_key_quiet<double>(solver_options, "cost_ratio").value_or(cost_ratio);
  discount_factor = parse_key_quiet<double>(solver_options, "discount_factor").value_or(discount_factor);
  verbose         = parse_key_quiet<bool>(solver_options, "verbose").value_or(verbose);
  debug_print     = parse_key_quiet<bool>(solver_options, "debug_print").value_or(debug_print);
  threads         = parse_key_quiet<int>(solver_options, "threads").value_or(threads);
  use_tree_search = parse_key_quiet<bool>(solver_options, "use_tree_search").value_or(use_tree_search);
  pruning_threshold = parse_key_quiet<double>(solver_options, "pruning_threshold").value_or(pruning_threshold);
  expert_weights   = parse_key_quiet<Eigen::VectorXd>(solver_options, "expert_weights").value_or(expert_weights);
  expert_types 	   = {NORM, IMP};
  display_update_freq = parse_key_quiet<bool>(solver_options, "display_update_freq").value_or(display_update_freq);
//clang-format on

  if (parsing_error) return false;
  std::cout << "Solver options correctly parsed from: " << file << std::endl;
  std::cout << *this << std::endl;
  return true;
}

std::ostream &operator<<(std::ostream &os, const mppi::SolverConfig &config) {
  os << "--------------------------------------------------" << std::endl;
  os << "Path integral solver configuration:" << std::endl;

  os << "General: " << std::endl;
  os << " rollouts:         " << config.rollouts << std::endl;
  os << " lambda:           " << config.lambda << std::endl;
  os << " h:                " << config.h << std::endl;
  os << " substeps:         " << config.substeps << std::endl;
  os << " adaptive sampling " << config.adaptive_sampling << std::endl;
  os << " input_variance:   " << config.input_variance.transpose() << std::endl; 
  
  os << " caching_factor:   " << config.caching_factor << std::endl;
  os << " step_size:        " << config.step_size << "s" << std::endl;
  os << " horizon:          " << config.horizon << "s" << std::endl;
  os << " steps:            " << std::floor(config.horizon / config.step_size) << std::endl;

  os << "Input constraints: " << std::endl;
  os << " bound input: " << config.bound_input << std::endl;
  os << " u min: " << config.u_min.transpose() << std::endl;
  os << " u max: " << config.u_max.transpose() << std::endl;
  
  os << "Filter: " << std::endl;
  os << " filter type:      " << config.filter_type << std::endl;
  os << " filter window:    " << config.filter_window << std::endl;
  os << " filter order:     " << config.filter_order << std::endl;
  os << " filtering:        " << config.filtering << std::endl;
  os << " cost ratio:       " << config.cost_ratio << std::endl;

  os << "Other stuff: " << std::endl;
  os << " discount factor:  " << config.discount_factor << std::endl;
  os << " verbose:          " << config.verbose << std::endl;
  os << " debug_print:      " << config.debug_print << std::endl;
  os << " threads:          " << config.threads << std::endl;
  os << "--------------------------------------------------" << std::endl;
  return os;
}