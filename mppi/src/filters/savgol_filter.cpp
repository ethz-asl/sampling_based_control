/*!
 * @file     savgol_filter.cpp
 * @author   Giuseppe Rizzi
 * @date     27.07.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi/filters/savgol_filter.h"

std::ostream& operator<<(std::ostream& os, const mppi::MovingExtendedWindow& w){
  os << "\nuu: [";
  for (size_t i=0; i<w.uu.size(); i++){
    os << w.uu[i] << " ";
  }
  os << "]\ntt: [";
  for (size_t i=0; i<w.uu.size(); i++){
    os << w.tt[i] << " ";
  }
  os << "]" << std::endl;
  return os;
}
