//
// Created by giulio on 04.11.21.
//

#pragma once
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <iostream>
#include <optional>
#include <string>

namespace manipulation {

/**
  Structure with all the configuration parameters for the MPPI solver
**/
struct Config {
 public:
  bool parsing_error = false;
  Eigen::VectorXd default_pose;
  double object_tolerance;
  std::string references_file;

  bool init_from_file(const std::string& file);

  template <typename T>
  static std::optional<T> parse_key(const YAML::Node& node, const std::string& key, bool& success_flag);
};

template <typename T>
std::optional<T> Config::parse_key(const YAML::Node& node,
                                   const std::string& key,
                                   bool& success_flag) {

  if (not node[key]) {
    std::cout << "Could not find entry: " << key << std::endl;
    success_flag = false;
//    if (!quiet) parsing_error = true;
    return {};
  }else {
    return node[key].as<T>();
  }
};

template <>
inline std::optional<Eigen::VectorXd> Config::parse_key<Eigen::VectorXd>(
    const YAML::Node& node, const std::string& key, bool& success_flag) {

  if (not node[key]) {
    std::cout << "Could not find entry: " << key << std::endl;
    success_flag = false;
//    if (!quiet) parsing_error = true;
    return {};
  }else {
    auto v = node[key].as<std::vector<double>>();
    Eigen::VectorXd v_eigen(v.size());
    for (size_t i = 0; i < v.size(); i++) v_eigen(i) = v[i];
    return v_eigen;
  }
};

}  // namespace manipulation
