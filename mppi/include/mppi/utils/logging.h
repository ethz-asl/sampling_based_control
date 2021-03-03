/*!
 * @file     logging.h
 * @author   Giuseppe Rizzi
 * @date     11.08.2020
 * @version  1.0
 * @brief    description
 */
#pragma once
#include <chrono>
#include <iostream>

using namespace std::chrono;

namespace mppi {

// TODO how can I define macros for print?

struct Timer {
  Timer() { reset(); }
  time_point<steady_clock> t;
  void reset() noexcept { t = steady_clock::now(); }
  inline double elapsed() {
    return (duration_cast<milliseconds>(steady_clock::now() - t).count() /
            1000.0);
  }
};

void log_info(const std::string& msg) {
  std::cout << "[PI_INFO]  : " << msg << std::endl;
}

void log_warning(const std::string& msg) {
  std::cout << "\033[1;93m[PI_WARN]  : " << msg + "\033[0m" << std::endl;
}

void log_error(const std::string& msg) {
  std::cout << "\033[1;31m[PI_ERROR] : " << msg + "\033[0m" << std::endl;
}

void log_warning_throttle(const double dt, const std::string& msg) {
  static Timer timer;
  static bool first_run = true;
  if (timer.elapsed() > dt || first_run) {
    log_warning(msg);
    timer.reset();
    first_run = false;
  }
}

}  // namespace mppi