/*!
 * @file     savgol_filter.h
 * @author   Giuseppe Rizzi
 * @date     27.07.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#include "gram_savitzky_golay.h"
#include <boost/circular_buffer.hpp>
#include <deque>
#include <iomanip>
#include <iostream>
#include <vector>
#include <map>
#include <iostream>

namespace mppi {
struct MovingExtendedWindow;
class ExactMovingExtendedWindow;
}

std::ostream& operator<<(std::ostream& os, const mppi::MovingExtendedWindow& w);

std::ostream& operator<<(std::ostream& os, const mppi::ExactMovingExtendedWindow& emew);

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

  void set(const double u, const double t) {
    auto upper = std::upper_bound(
        tt.begin(), tt.end(), t);  // index to the first element larger than t
    assert(lower != tt.end());
    size_t idx = std::distance(tt.begin(), upper) - 1;
    uu[idx] = u;
  }

  /**
   * Extend the window until the end with the latest received measurement
   */
  void extend() {
    std::fill(uu.begin() + start_idx + 1, uu.end(), uu[start_idx]);
    std::fill(tt.begin() + start_idx + 1, tt.end(), tt[start_idx]);
  }
};

class ExactMovingExtendedWindow {
  public: 
    ExactMovingExtendedWindow(const int size, const int w, const double dt) {
      w_ = w;
      dt_ = dt * 1000;
      for (int i=0; i<size + 2 * w_ + 1; i++){
        u_[i*dt_ - w_*dt_] = 0;
      }
      
      last_trim_t_ = -1;
    }

  public:
    double dt_;
    int w_;
    double last_trim_t_;
    std::map<double, double> u_;
  
  public:

    void trim(const double t) {
      int t_ms = (int)(t * 1000);
      std::cout << "Trimming at " << t_ms << std::endl;
      std::cout << *this;

      if (t_ms < last_trim_t_) {
        std::stringstream ss;
        ss << "Resetting the window back in the past. Can reset only to larger "
              "times than last "
              "reset!!!"
           << "last reset=" << last_trim_t_ << ", trying to reset to t=" << t;
        throw std::runtime_error(ss.str());
      }
      last_trim_t_ = t_ms;

      double new_first_time = last_trim_t_ - w_ * dt_;

      u_.erase(u_.begin(), u_.find(new_first_time));

      // extend with the last input
      double last_time = std::next(u_.end(),-1)->first;
      for (int i=0; i<w_; i++){
        u_[last_time + i*dt_] = u_[last_time];
      }

    }

    void add_point(const double u, const double t) { 
      int t_ms = (int)(t*1000);
      u_[t_ms] = u; 

      // extend with the newly added value
      for (auto it=std::next(u_.find(t_ms), 1); it != u_.end(); it++){
        it->second = u;
      }
    }

    /**
     * Extract a window centered at the query time
     * @param t: query time
     * @return
     */
    std::vector<double> extract(const double t) {
      int t_ms = (int)(t*1000);
      std::vector<double> v;
      v.resize(2*w_ + 1);
      v[w_] = u_[t_ms];

      for (int i=0; i<w_; i++){
        v[i] = u_[t_ms - (w_-i)*dt_];
        v[2*w_ - i] = u_[t_ms + (w_ + i)*dt_];
      }
      
      return v;  
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
  void apply(Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<> > u, const double t);
  inline std::vector<MovingExtendedWindow>& get_windows() { return windows_; }

 private:
  int window_size_;
  gram_sg::SavitzkyGolayFilter filter;

  std::vector<MovingExtendedWindow> windows_;
  std::vector<gram_sg::SavitzkyGolayFilter> filters_;
};

}  // namespace mppi
