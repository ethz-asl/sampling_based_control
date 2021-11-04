//
// Created by giulio on 04.11.21.
//
#include <cmath>
#include <iostream>

#include "mppi_manipulation/config_no_ros.h"

using namespace manipulation;

bool Config::init_from_file(const std::string& file) {
  bool success_flag = true;
  YAML::Node config;
  try {
    config = YAML::LoadFile(file);
  } catch (const YAML::ParserException& ex) {
    std::cout << ex.what() << std::endl;
  } catch (const YAML::BadFile& ex) {
    std::cout << ex.what() << std::endl;
  }

  YAML::Node options = config["options"]; // todo: GIS change this according to config structure
  if (!options) {
    std::cout << "Failed to parse solver options." << std::endl;
    return false;
  }
  default_pose = parse_key<Eigen::VectorXd>(options, "default_pose", success_flag).value_or(Eigen::VectorXd(0));
  object_tolerance = parse_key<double>(options, "object_tolerance", success_flag).value_or(0.0);
  references_file = parse_key<std::string>(options, "references_file", success_flag).value_or("");

  return success_flag;
}