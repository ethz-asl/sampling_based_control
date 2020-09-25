/*!
 * @file     savgol_filter.h
 * @author   Giuseppe Rizzi
 * @date     27.07.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#include <deque>
#include "mppi/filters/filter_base.h"
#include <gram_savitzky_golay/gram_savitzky_golay.h>
#include <boost/circular_buffer.hpp>

namespace mppi{

class SavGolFilter : public FilterBase{
 public:
  using filter_ptr = FilterBase::filter_ptr;

  SavGolFilter() = default;
  SavGolFilter(const int channels, const int window, const uint poly_order, const uint der_order=0, const double time_step=1.){
    sgf = gram_sg::SavitzkyGolayFilter(window, 0, poly_order, der_order);
    channels_window.resize(channels);
    // the actual window size is 2*window + 1
    for(auto& channel : channels_window){
      channel.resize(window*2 + 1, 0.0);
    }
    window_ = window;
  };
  ~SavGolFilter() = default;

 public:
  int window() override { return window_; }

  void filter(Eigen::VectorXd& channels) override {
    for (size_t i=0; i<channels.size(); i++){
      channels_window[i].push_back(channels(i));
      channels(i) = sgf.filter(channels_window[i]);
    }
  }

  filter_ptr clone() override { return std::make_shared<SavGolFilter>(*this); }

 private:
  int window_;
  std::vector<boost::circular_buffer<double>> channels_window;
  gram_sg::SavitzkyGolayFilter sgf;
};

}
