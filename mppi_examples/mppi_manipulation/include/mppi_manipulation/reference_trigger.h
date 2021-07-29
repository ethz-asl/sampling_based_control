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

class ReferenceTrigger {
  void add_reference(const double t, const mppi::reference_t& ref) {
    if (t < times_.back()) {
      std::cout
          << "[ReferenceTrigger] Trying to add a reference back in the past."
          << std::endl;
      return;
    }
    mppi::reference_trajectory_t rt;
    rt.rr.push_back(ref);
    rt.tt.push_back(t);
    schedule_.emplace_back(rt);
    times_.emplace_back(t);
  }

  bool has_reference(const double t) {
    return times_.empty() ? false : t > times_[0];
  }

  bool parse_from_file(const std::string& file_path) {
    YAML::Node config;
    try {
      config = YAML::LoadFile(file_path);
    } catch (const YAML::ParserException& ex) {
      std::cout << ex.what() << std::endl;
    } catch (const YAML::BadFile& ex) {
      std::cout << ex.what() << std::endl;
    }

    if (!config.IsSequence()) {
      std::cout << "[ReferenceTrigger] Failed to parse references file."
                << std::endl;
      return false;
    }

    for (const auto& element : config) {
      if (!element.IsMap()) {
        std::cout << "[ReferenceTrigger] Element should be dictionary"
                  << std::endl;
      }
      auto r = element["r"].as<std::vector<double>>();
      auto t = element["t"].as<double>();
      mppi::reference_t r_eigen;
      r_eigen.setZero(r.size());
      for (int i = 0; i < r.size(); i++) r_eigen(i) = r[i];
      add_reference(t, r_eigen);
    }
  }

  void set_reference(const double t, mppi::reference_trajectory_t& rt) {
    if (t > times_[0]) {
      auto lower = std::lower_bound(times_.begin(), times_.end(), t);
      size_t index = std::distance(times_.begin(), lower);
      rt = schedule_[index - 1];
    }
  }

 private:
  std::vector<double> times_;
  std::vector<mppi::reference_trajectory_t> schedule_;
};

}  // namespace manipulation
