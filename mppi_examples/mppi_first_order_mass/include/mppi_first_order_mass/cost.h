/*!
 * @file     pendulum_cart_cost.h
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#pragma once
#include <mppi/core/cost.h>
#include <cmath>

namespace fom {

class FOMCost : public mppi::Cost {
 public:
  FOMCost() = default;
  ~FOMCost() = default;

 public:
  mppi::cost_ptr create() override { return std::make_shared<FOMCost>(); }

  mppi::cost_ptr clone() const override {
    return std::make_shared<FOMCost>(*this);
  }

  // When the limit is violeted than the cost is only almost the max cost.
  // adding an increasing cost can push the agent to enter the violated limits
  // again
  mppi::cost_t compute_cost(const mppi::observation_t& x,
                            const mppi::input_t& u,
                            const mppi::reference_t& ref,
                            const double t) override {
    return (ref - x).norm() + 0.01 * u.norm();
  }
};
}  // namespace fom
