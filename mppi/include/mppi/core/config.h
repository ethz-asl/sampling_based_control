/*!
 * @file     solver_config.h
 * @author   Giuseppe Rizzi
 * @date     01.07.2020
 * @version  1.0
 * @brief    description
 */
#pragma once
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <iostream>
#include <optional>
#include <string>

namespace mppi {

/**
  Structure with all the configuration parameters for the MPPI solver
**/
struct Config {
  size_t rollouts = 1;
  double lambda = 1.0;
  double h = 1.0;
  double step_size = 0.1;
  double horizon = 1.0;
  double caching_factor = 0.0;
  size_t substeps = 1;

  double alpha = 1.0;
  double beta = 0.0;
  bool adaptive_sampling = false;
  Eigen::VectorXd input_variance;

  double cost_ratio = 0.2;
  double discount_factor = 1.0;

  bool verbose = true;
  bool debug_print = false;

  bool bound_input;
  Eigen::VectorXd u_min;
  Eigen::VectorXd u_max;

  bool filtering = false;
  std::vector<int> filters_type;
  std::vector<int> filters_window;
  std::vector<uint> filters_order;

  size_t threads = 1;
  bool logging = false;

  int spline_degree = 3;
  double spline_dt = 0.15;
  double spline_verbose = false;
  double spline_step_size = 0.1;

  bool display_update_freq = false;

  bool init_from_file(const std::string& file);

 private:
  bool parsing_error = false;
  template <typename T>
  std::optional<T> parse_key(const YAML::Node& node, const std::string& key,
                             bool quiet = false);

  template <typename T>
  std::optional<T> parse_key_quiet(const YAML::Node& node,
                                   const std::string& key);
};

template <typename T>
std::optional<T> Config::parse_key(const YAML::Node& node,
                                   const std::string& key, bool quiet) {
  if (!node[key]) {
    std::cout << "Could not find entry: " << key << std::endl;
    if (!quiet) parsing_error = true;
    return {};
  }
  return node[key].as<T>();
};

template <>
inline std::optional<Eigen::VectorXd> Config::parse_key<Eigen::VectorXd>(
    const YAML::Node& node, const std::string& key, bool quiet) {
  if (!node[key]) {
    std::cout << "Could not find entry: " << key << std::endl;
    if (!quiet) parsing_error = true;
    return {};
  }
  auto v = node[key].as<std::vector<double>>();
  Eigen::VectorXd v_eigen(v.size());
  for (size_t i = 0; i < v.size(); i++) v_eigen(i) = v[i];

  return v_eigen;
};

template <typename T>
std::optional<T> Config::parse_key_quiet(const YAML::Node& node,
                                         const std::string& key) {
  return parse_key<T>(node, key, true);
};

}  // namespace mppi

std::ostream& operator<<(std::ostream& os, const mppi::Config& config);
