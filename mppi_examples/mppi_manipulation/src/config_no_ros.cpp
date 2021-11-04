//
// Created by giulio on 04.11.21.
//
#include <cmath>
#include <iostream>

#include "mppi_manipulation/config_no_ros.h"

using namespace manipulation;

bool Config::init_from_file(const std::string& file) {
  bool sf = true; // success flag -> turned to false if something goes wrong during YAML parsing
  try {
    data = YAML::LoadFile(file);
  } catch (const YAML::ParserException& ex) {
    std::cout << ex.what() << std::endl;
  } catch (const YAML::BadFile& ex) {
    std::cout << ex.what() << std::endl;
  }

  YAML::Node options = data["options"];
  if (!options) {
    std::cout << "Failed to parse solver options." << std::endl;
    return false;
  }
  // general params
  default_pose = parse_key<Eigen::VectorXd>(options, "default_pose", sf).value_or(Eigen::VectorXd(0));
  object_tolerance = parse_key<double>(options, "object_tolerance", sf).value_or(0.0);
  references_file = parse_key<std::string>(options, "references_file", sf).value_or("");
  controller_config_file = parse_key<std::string>(options, "controller_config_file", sf).value_or("");
  gaussian_policy = parse_key<bool>(options, "gaussian_policy", sf).value_or(false);

  // dynamics
  YAML::Node dynamics = data["dynamics"];
  if (!dynamics) {
    std::cout << "Failed to parse dynamics." << std::endl;
    return false;
  }

  // these values are parsed here in order to have better access from python, if needed
  dt = parse_key<double>(dynamics, "dt", sf).value_or(0.015);
  robot_description = parse_key<std::string>(dynamics, "robot_description", sf).value_or("");
  object_description = parse_key<std::string>(dynamics, "object_description", sf).value_or("");
  gains = PIDGains(); // initialize gains to default value, lets me skip another config parsing
  initial_state = parse_key<Eigen::VectorXd>(dynamics, "initial_state", sf).value_or(Eigen::VectorXd(0));

  articulation_joint = parse_key<std::string>(dynamics, "articulation_joint", sf).value_or("");
  object_handle_link = parse_key<std::string>(dynamics, "object_handle_link", sf).value_or("");
  object_handle_joint = parse_key<std::string>(dynamics, "object_handle_joint", sf).value_or("");

//  std::vector<double> arm_gains = parse_key<std::vector<double>>(dynamics, "arm_gains", success_flag).value_or(std::vector<double>{0, 10, 0, 1});
//  std::vector<double> base_gains = parse_key<std::vector<double>>(dynamics, "base_gains", success_flag).value_or(std::vector<double>{0, 1000, 0, 0});
//  std::vector<double> gripper_gains = parse_key<std::vector<double>>(dynamics, "gripper_gains", success_flag).value_or(std::vector<double>{100, 50, 0, 0});

  return sf;
}