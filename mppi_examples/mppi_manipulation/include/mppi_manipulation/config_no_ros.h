//
// Created by giulio on 04.11.21.
//

#pragma once
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <iostream>
#include <optional>
#include <string>
#include "mppi_manipulation/params/gains.h"

namespace manipulation {

/**
  Structure with all the configuration parameters for the MPPI solver
**/
struct Config {
 public:
  YAML::Node data;
  // general params
  bool parsing_error = false;
  Eigen::VectorXd default_pose;
  double object_tolerance;
  std::string references_file;
  std::string solver_config_file;
  bool gaussian_policy;

  // dynamics
  double dt;
  std::string robot_description;
  std::string object_description;
  std::string articulation_joint;
  std::string object_handle_link;
  std::string object_handle_joint;

  PIDGains gains;
  Eigen::VectorXd initial_state;

  bool init_from_file(const std::string& file);

  template <typename T>
  static std::optional<T> parse_key(const YAML::Node& node, const std::string& key, bool& success_flag);
};

template <typename T>
std::optional<T> Config::parse_key(const YAML::Node& node,
                                   const std::string& key,
                                   bool& success_flag) {

  if (not node[key]) {
    std::cout << "Manipulation Config - could not find entry: " << key << std::endl;
    success_flag = false;
    return {};
  }else {
    return node[key].as<T>();
  }
};

template <>
inline std::optional<Eigen::VectorXd> Config::parse_key<Eigen::VectorXd>(
    const YAML::Node& node, const std::string& key, bool& success_flag) {

  if (not node[key]) {
    std::cout << "Manipulation Config - could not find entry: " << key << std::endl;
    success_flag = false;
    return {};
  }else {
    auto v = node[key].as<std::vector<double>>();
    Eigen::VectorXd v_eigen(v.size());
    for (size_t i = 0; i < v.size(); i++) v_eigen(i) = v[i];
    return v_eigen;
  }
};

}  // namespace manipulation
