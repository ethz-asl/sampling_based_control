/*!
 * @file     savgol_filter.h
 * @author   Giuseppe Rizzi
 * @date     27.07.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#include <mppi/filters/gram_savitzky_golay.h>
#include <boost/circular_buffer.hpp>
#include <deque>
#include <iomanip>
#include <iostream>
#include <vector>

namespace mppi {
struct MovingExtendedWindow;
}

std::ostream& operator<<(std::ostream& os, const mppi::MovingExtendedWindow& w);

namespace mppi {

struct MovingExtendedWindow {
  MovingExtendedWindow(const int size, const int w) {
    window = w;
    // the first and last input need half horizon in the past and in the future
    // in order to be centered leaving with a total size equal to weigths size
    // (2 * window) + 1 + horizon/2
    // + horizon/2 = 2 * window + 1 + horizon
    uu.resize(size + 2 * window + 1, 0);

    // initialization to -1 makes sure everything starts correctly with a
    // positive initial time
    tt.resize(size + 2 * window + 1, -1);

    start_idx = window;
    last_trim_t = -1;
  }

  double last_trim_t;
  int start_idx;
  int window;
  std::vector<double> uu;
  std::vector<double> tt;

  void trim(const double t) {
    if (t < last_trim_t) {
      std::stringstream ss;
      ss << "Resetting the window back in the past. Can reset only to larger "
            "times than last "
            "reset!!!"
         << "last reset=" << last_trim_t << ", trying to reset to t=" << t;
      throw std::runtime_error(ss.str());
    }
    last_trim_t = t;

    // search in the last inserted times the closest smaller than the current
    size_t trim_idx = start_idx;
    for (size_t i = 0; i < start_idx; i++) {
      if (tt[i] >= t) {
        trim_idx = i;
        break;
      }
    }
    size_t offset = trim_idx - window;

    std::rotate(tt.begin(), tt.begin() + offset, tt.end());
    std::rotate(uu.begin(), uu.begin() + offset, uu.end());

    // extend the trimmed portion with the last elements in the vector
    if (offset > 0) {
      std::fill(tt.end() - offset, tt.end(), *(tt.end() - offset - 1));
      std::fill(uu.end() - offset, uu.end(), *(uu.end() - offset - 1));
    }

    start_idx = window;
    tt[start_idx] = t;
  }

  void add_point(const double u, const double t) {
    if (t < tt[start_idx]) {
      std::stringstream ss;
      ss << std::setprecision(4)
         << "Adding measurement older then new time: " << t << " < "
         << tt[start_idx] << std::endl;
      ss << "start_idx: " << start_idx << std::endl;
      ss << "last trim time: " << last_trim_t << std::endl;
      ss << "window: " << *this << std::endl;
      throw std::runtime_error(ss.str());
    }
    assert(start_idx < uu.size());
    uu[start_idx] = u;
    tt[start_idx] = t;

    extend();
    start_idx++;
  }

  /**
   * Extract a window centered at the query time
   * @param t: query time
   * @return
   */
  std::vector<double> extract(const double t) {
    auto lower = std::lower_bound(
        tt.begin(), tt.end(), t);  // index to the first element larger than t
    assert(lower != tt.end());
    size_t idx = std::distance(tt.begin(), lower);

    return std::vector<double>(uu.begin() + idx - window,
                               uu.begin() + idx + window + 1);
  }

  /**
   * Extend the window until the end with the latest received measurement
   */
  void extend() {
    std::fill(uu.begin() + start_idx + 1, uu.end(), uu[start_idx]);
    std::fill(tt.begin() + start_idx + 1, tt.end(), tt[start_idx]);
  }
};

class SavGolFilter {
 public:
  SavGolFilter() = default;
  SavGolFilter(const int steps, const int nu, const int window,
               const uint poly_order, const uint der_order = 0,
               const double time_step = 1.);
  /**
  Filter with custom filtering per input channel
  **/
  SavGolFilter(const int steps, const int nu, const std::vector<int>& window,
               const std::vector<uint>& poly_order, const uint der_order = 0,
               const double time_step = 1.);

  ~SavGolFilter() = default;

 public:
  void reset(const double t);
  void add_measurement(const Eigen::VectorXd& u, const double t);
  void apply(Eigen::VectorXd& u, const double t);
  inline std::vector<MovingExtendedWindow>& get_windows() { return windows_; }

 private:
  int window_size_;
  gram_sg::SavitzkyGolayFilter filter;

  std::vector<MovingExtendedWindow> windows_;
  std::vector<gram_sg::SavitzkyGolayFilter> filters_;
};

}  // namespace mppi
