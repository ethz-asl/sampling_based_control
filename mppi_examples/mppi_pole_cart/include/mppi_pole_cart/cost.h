/*!
 * @file     pendulum_cart_cost.h
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#pragma once
#include <math.h>
#include <mppi/cost/cost_base.h>

namespace pole_cart {

class PoleCartCost : public mppi::CostBase {
 public:
  PoleCartCost() = default;
  ~PoleCartCost() = default;

 private:
  double w_theta = 10;
  double w_origin = 1;
  double c_x_limit = 10000;
  double x_limit = 3.0;

 public:
  cost_ptr create() override { return std::make_shared<PoleCartCost>(); }

  cost_ptr clone() const override {
    return std::make_shared<PoleCartCost>(*this);
  }

  // When the limit is violeted than the cost is only almost the max cost.
  // adding an increasing cost can push the agent to enter the violated limits
  // again
  cost_t compute_cost(const mppi::observation_t& x,
                      const mppi::reference_t& ref, const double t) override {
    double cost = 0.0;
    if (x(0) > x_limit || x(0) < -x_limit)
      cost += c_x_limit * (1 + w_origin * ((std::abs(x(0)) - x_limit) *
                                           (std::abs(x(0)) - x_limit)));

    double theta = std::fmod(x(1), 2 * M_PI);
    double delta = (theta < 0) ? ref(1) + theta : ref(1) - theta;
    cost += w_theta * std::pow(delta, 2);

    if (std::abs(delta) < 10.0 * ref(1) / 180.0) {
      cost += w_origin * std::pow(x(0) - ref(0), 2);
    }

    return cost;
  }
};
}  // namespace pole_cart
