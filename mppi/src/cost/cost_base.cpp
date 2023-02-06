/*!
 * @file     cost_base.cpp
 * @author   Giuseppe Rizzi
 * @date     01.07.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi/cost/cost_base.h"
#include <iostream>

namespace mppi {

CostBase::CostBase(const CostBase &cost_base) {
  std::shared_lock lock(cost_base.timed_reference_mutex_);
  timed_ref_ = cost_base.timed_ref_;
  r_ = cost_base.r_;
}

void CostBase::set_reference_trajectory(const reference_trajectory_t &traj) {
  if (traj.rr.size() > 0 && traj.tt.size() > 0) {
    std::scoped_lock lock(timed_reference_mutex_);

    timed_ref_.tt.clear();
    timed_ref_.tt.reserve(traj.tt.size());
    timed_ref_.rr.clear();
    for (size_t i = 0; i < traj.tt.size(); i++) {
      timed_ref_.rr.push_back(traj.rr[i]);
      timed_ref_.tt.push_back(traj.tt[i]);
    }
  }
}

void CostBase::interpolate_reference(const observation_t & /*x*/,
                                     reference_t &ref, const double t) {
  std::shared_lock lock(timed_reference_mutex_);

  auto lower = std::lower_bound(timed_ref_.tt.begin(), timed_ref_.tt.end(), t);
  size_t offset = std::distance(timed_ref_.tt.begin(), lower);
  if (lower == timed_ref_.tt.end() && offset > 0) offset -= 1;
  ref = timed_ref_.rr.at(offset);
}

CostBase::cost_t CostBase::get_stage_cost(const mppi::observation_t &x,
                                          const double t) {
  interpolate_reference(x, r_, t);
  return compute_cost(x, r_, t);
}

}  // namespace mppi