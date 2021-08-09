//
// Created by giuseppe on 29.07.21.
//

#pragma once
#include <mppi/core/typedefs.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <utility>
#include <vector>

namespace manipulation {

class ReferenceScheduler {
 public:
  void add_reference(const double t, const mppi::reference_t& ref);
  bool has_reference(const double t);
  bool parse_from_file(const std::string& file_path);
  void set_reference(const double t, mppi::reference_trajectory_t& rt);

 private:
  int reference_counter_ = 0;
  std::vector<double> times_;
  std::vector<mppi::reference_trajectory_t> schedule_;
};

}  // namespace manipulation
