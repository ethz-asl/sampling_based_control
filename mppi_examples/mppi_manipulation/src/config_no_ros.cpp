//
// Created by giulio on 04.11.21.
//
#include <cmath>
#include <iostream>
#include <fstream>

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
  solver_config_file = parse_key<std::string>(options, "solver_config_file", sf).value_or("");
  gaussian_policy = parse_key<bool>(options, "gaussian_policy", sf).value_or(false);
  debug_prints = parse_key<bool>(options, "debug_prints", sf).value_or(true);

  // dynamics
  YAML::Node dynamics = data["dynamics"];
  if (!dynamics) {
    std::cout << "Failed to parse dynamics." << std::endl;
    return false;
  }

  // these values are parsed here in order to have better access from python, if needed
  dt = parse_key<double>(dynamics, "dt", sf).value_or(0.015);
  robot_description = get_model(dynamics, "robot_description");
  object_description = get_model(dynamics, "object_description");
  robot_description_raisim = get_model(dynamics, "robot_description_raisim");
  object_description_raisim = get_model(dynamics, "object_description_raisim");
  raisim_object_res_path = parse_key<std::string>(dynamics, "raisim_object_res_path", sf).value_or("");
  raisim_robot_res_path = parse_key<std::string>(dynamics, "raisim_robot_res_path", sf).value_or("");
  ignore_object_self_collision = parse_key<bool>(dynamics, "ignore_object_self_collision", sf).value_or(true);
  ignore_panda_self_collision = parse_key<bool>(dynamics, "ignore_panda_self_collision", sf).value_or(true);

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

std::string Config::get_model(YAML::Node yaml_mode, std::string key){
  bool sf = true;
  std::string description_path = parse_key<std::string>(yaml_mode, key, sf).value_or("");
  if(not sf){
    std::cout << "Could not load model path <" << key << "> from yaml node" << std::endl;
  }
  // need to load the file manually to string
  std::ifstream file_stream(description_path);
  std::string model(std::istreambuf_iterator<char>{file_stream}, {});
  return model;
}