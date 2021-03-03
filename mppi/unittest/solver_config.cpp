/*!
 * @file     solver_config.cpp
 * @author   Giuseppe Rizzi
 * @date     27.07.2020
 * @version  1.0
 * @brief    description
 */
#pragma once
#include "mppi/solver_config.h"
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

using namespace mppi;

TEST(SolverConfig, Parser) {
  std::string file_path = __FILE__;
  std::string dir_path = file_path.substr(0, file_path.rfind('/'));
  std::string config_file = dir_path + "/resources/config.yaml";
  std::cout << "Parsing solver options from: " << config_file << std::endl;

  SolverConfig config;
  config.init_from_file(config_file);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}