/*!
 * @file     savgol_filter.cpp
 * @author   Giuseppe Rizzi
 * @date     27.07.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi/utils/savgol_filter.h"

std::ostream& operator<<(std::ostream& os,
                         const mppi::MovingExtendedWindow& w) {
  os << "\nuu: [";
  for (size_t i = 0; i < w.uu.size(); i++) {
    os << w.uu[i] << " ";
  }
  os << "]\ntt: [";
  for (size_t i = 0; i < w.uu.size(); i++) {
    os << w.tt[i] << " ";
  }
  os << "]" << std::endl;
  return os;
}

std::ostream& operator<<(std::ostream& os, const mppi::ExactMovingExtendedWindow& emew){
   os << "Window is: [";
   for (auto it=emew.u_.begin(); it!=emew.u_.end(); it++){
       os << "(" << it->first << ", " << it->second << ") "; 
   }
   os << "]" << std::endl;
   return os; 
};

namespace mppi {

SavGolFilter::SavGolFilter(const int steps, const int nu, const int window,
                           const uint poly_order, const uint der_order,
                           const double time_step) {
  filters_.resize(
      nu, gram_sg::SavitzkyGolayFilter(window, 0, poly_order, der_order));
  windows_.resize(nu, MovingExtendedWindow(steps, window));
};

SavGolFilter::SavGolFilter(const int steps, const int nu,
                           const std::vector<int>& window,
                           const std::vector<uint>& poly_order,
                           const uint der_order, const double time_step) {
  if (window.size() == 1) {
    filters_.resize(nu, gram_sg::SavitzkyGolayFilter(window[0], 0,
                                                     poly_order[0], der_order));
    windows_.resize(nu, MovingExtendedWindow(steps, window[0]));
    return;
  }
  for (int i = 0; i < nu; i++) {
    filters_.emplace_back(window[i], 0, poly_order[i], der_order);
    windows_.emplace_back(steps, window[i]);
  }
};

void SavGolFilter::reset(const double t) {
  for (auto& w : windows_) w.trim(t);
}

void SavGolFilter::add_measurement(const Eigen::VectorXd& u, const double t) {
  assert(u.size() == windows_.size());
  for (long int i = 0; i < u.size(); i++) {
    windows_[i].add_point(u(i), t);
  }
}

void SavGolFilter::apply(Eigen::Ref<Eigen::VectorXd, 0, Eigen::InnerStride<>> u, const double t) {
  for (long int i = 0; i < u.size(); i++) {
    u[i] = filters_[i].filter(windows_[i].extract(t));
    windows_[i].set(u[i], t); // update the window with the filtered value
  }
}


}  // namespace mppi
