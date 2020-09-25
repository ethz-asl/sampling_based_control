/*!
 * @file     solver_config.h
 * @author   Giuseppe Rizzi
 * @date     01.07.2020
 * @version  1.0
 * @brief    description
 */
#pragma once
#include <optional>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace mppi {

enum InputFilterType : char {
  NONE = 0,
  SAVITZKY_GOLEY = 1,
};

struct SolverConfig {

  int rollouts = 1;
  double lambda = 1.0;
  double h = 1.0;
  double step_size = 0.1;
  double horizon = 1.0;
  double caching_factor = 0.0;
  int substeps = 1.0;

  bool adaptive_sampling = false;
  std::vector<double> input_variance;

  bool filtering = false;
  double cost_ratio = 0.2;
  double discount_factor = 1.0;

  bool verbose = true;
  bool debug_print = false;
  bool use_gui = false;

  InputFilterType filter_type = InputFilterType::NONE;
  uint filter_window = 10;
  uint filter_order = 3;

  size_t threads = 1;

  bool init_from_file(const std::string& file);
 private:
  bool parsing_error = false;
  template<typename T>
  std::optional<T> parse_key(const YAML::Node& node, const std::string& key, bool quiet=false);

  template<typename T>
  std::optional<T> parse_key_quiet(const YAML::Node& node, const std::string& key);
};

template<typename T>
std::optional<T> SolverConfig::parse_key(const YAML::Node& node, const std::string &key, bool quiet) {
  if (!node[key]){
    std::cout << "Could not find entry: " << key << std::endl;
    if (quiet) parsing_error = true;
    return {};
  }
  return node[key].as<T>();
}

template<typename T>
std::optional<T> SolverConfig::parse_key_quiet(const YAML::Node& node, const std::string &key) {
  return parse_key<T>(node, key, true);
}

}

std::ostream &operator<<(std::ostream &os, const mppi::SolverConfig &config);
