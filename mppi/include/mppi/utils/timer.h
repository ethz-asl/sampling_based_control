//
// Created by giuseppe on 01.02.21.
//

#pragma once

#include <chrono>
#include <iostream>
#include <map>

class Timer {
 public:
  using time_t = std::chrono::time_point<std::chrono::steady_clock>;

  void reset() {
    current_ = std::chrono::steady_clock::now();
    since_last_interval_ = current_;
  }
  void add_interval(const std::string& name) {
    time_t now = std::chrono::steady_clock::now();
    intervals_[name] = std::chrono::duration<double>(now - since_last_interval_);
    since_last_interval_ = now;
  }

  void print_intervals() {
    for (const auto& interval : intervals_) {
      std::cout << "Interval [" << interval.first << "] took: "
                << std::chrono::duration_cast<std::chrono::nanoseconds>(interval.second).count() / 1e6
                << " ms." << std::endl;
    }
  }

 private:
  time_t current_;
  time_t since_last_interval_;
  std::map<std::string, std::chrono::duration<double>> intervals_;
};