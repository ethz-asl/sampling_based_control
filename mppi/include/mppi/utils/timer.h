//
// Created by giuseppe on 01.02.21.
//

#pragma once

#include <chrono>
#include <cmath>
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
    if (!intervals_.count(name)) {
      intervals_count_[name] = 0;
      intervals_cumulated_[name] = 0.0;
    }

    time_t now = std::chrono::steady_clock::now();
    intervals_[name] = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           now - since_last_interval_)
                           .count() /
                       1e6;
    since_last_interval_ = now;

    intervals_count_[name]++;
    intervals_cumulated_[name] += intervals_[name];
  }

  void print_intervals(size_t decimation = 0) {
    static size_t counter = 0;
    counter++;
    if (std::fmod(counter, decimation) == 0) {
      for (const auto& interval : intervals_) {
        std::cout << "Interval [" << interval.first
                  << "] took: " << interval.second << " ms, average: "
                  << intervals_cumulated_[interval.first] /
                         intervals_count_[interval.first]
                  << " ms." << std::endl;
      }
      counter = 0;
    }
  }

 private:
  time_t current_;
  time_t since_last_interval_;
  std::map<std::string, int> intervals_count_;
  std::map<std::string, double> intervals_;
  std::map<std::string, double> intervals_cumulated_;
};