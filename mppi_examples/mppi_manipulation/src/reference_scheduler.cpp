//
// Created by giuseppe on 29.07.21.
//

#include "mppi_manipulation/reference_scheduler.h"
#include <ros/ros.h>

namespace manipulation {

void ReferenceScheduler::add_reference(const double t,
                                       const mppi::reference_t& ref) {
  if (!times_.empty() && t < times_.back()) {
    std::cout
        << "[ReferenceScheduler]: trying to add a reference back in the past."
        << std::endl;
    return;
  }
  mppi::reference_trajectory_t rt;
  rt.rr.push_back(ref);
  rt.tt.push_back(t);
  schedule_.push_back(rt);
  times_.push_back(t);
  std::stringstream ss;
  ss << "[ReferenceScheduler]: adding new reference." << std::endl;
  ss << "  ref  = [" << ref.transpose() << "]" << std::endl;
  ss << "  time = " << t << std::endl;
  std::cout << ss.str();
}

bool ReferenceScheduler::has_reference(const double t) {
  return times_.empty() ? false : t > times_[0];
}

bool ReferenceScheduler::parse_from_file(const std::string& file_path) {
  std::cout << "[ReferenceScheduler]: parsing references from " << file_path
            << std::endl;
  YAML::Node config;
  try {
    config = YAML::LoadFile(file_path);
  } catch (const YAML::ParserException& ex) {
    ROS_WARN_STREAM(ex.what());
  } catch (const YAML::BadFile& ex) {
    ROS_WARN_STREAM(ex.what());
    ;
  }

  if (!config.IsSequence()) {
    std::cout << "[ReferenceScheduler] Failed to parse references file."
              << std::endl;
    return false;
  }

  for (const auto& element : config) {
    if (!element.IsMap()) {
      std::cout << "[ReferenceScheduler]: element should be dictionary"
                << std::endl;
    }
    auto r = element["r"].as<std::vector<double>>();
    auto t = element["t"].as<double>();
    mppi::reference_t r_eigen;
    r_eigen = mppi::reference_t::Zero(r.size());
    r_eigen.setZero(r.size());
    for (int i = 0; i < r.size(); i++) r_eigen(i) = r[i];
    add_reference(t, r_eigen);
  }
  return true;
}

void ReferenceScheduler::set_reference(const double t,
                                       mppi::reference_trajectory_t& rt) {
  if (t > times_[0]) {
    auto lower = std::lower_bound(times_.begin(), times_.end(), t);
    size_t index = std::distance(times_.begin(), lower);
    rt = schedule_[index - 1];
    if ((index - 1) == reference_counter_) {
      std::cout << "[ReferenceScheduler]: setting reference "
                << reference_counter_ << std::endl;
      std::cout << rt.rr[0].transpose() << std::endl;
      std::cout << rt.tt[0] << std::endl;
      reference_counter_++;
    }
  }
}

}  // namespace manipulation
