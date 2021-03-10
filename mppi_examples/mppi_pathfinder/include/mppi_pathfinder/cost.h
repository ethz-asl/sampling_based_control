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

namespace pathfinder {

class PathfinderCost : public mppi::CostBase {
 public:
  PathfinderCost() = default;
  ~PathfinderCost() = default;

 private:
  double w_x = 10;
  double w_y = 5;
  double w_theta = 0;
  double c_leafing_field = 0;
  double x_fieldlimitpositive = 25.0;
  double x_fieldlimitnegative = -5.0;
  double y_fieldlimit = 5;
  double c_obstacle = 10000;

 public:
  cost_ptr create() override { return std::make_shared<PathfinderCost>(); }

  cost_ptr clone() const override {
    return std::make_shared<PathfinderCost>(*this);
  }

  // When the limit is violeted than the cost is only almost the max cost.
  // adding an increasing cost can push the agent to enter the violated limits
  // again
  cost_t compute_cost(const mppi::observation_t& x,
                      const mppi::reference_t& ref, const double t) override {
    double cost = 0.0;
    if (x(0) > x_fieldlimitpositive || x(0) < x_fieldlimitnegative)
      cost += c_leafing_field;
    if (x(1) > y_fieldlimit || x(1) < -y_fieldlimit)
      cost += c_leafing_field * (1 + w_y * ((std::abs(x(1)) - y_fieldlimit) *
                                           (std::abs(x(1)) - y_fieldlimit)));
    if (x(0) > 13){
      if (x(0) < 15){
        if (x(1) > 1){
          if (x(1) < 3){
              cost += c_obstacle;
            }
        }
      }
    }

    double theta = std::fmod(x(1), 2 * M_PI);
    double delta = (theta < 0) ? ref(2) + theta : ref(2) - theta;
    cost += w_theta * std::pow(delta, 2);

    cost += w_x * std::pow(x(0) - ref(0), 2);
    cost += w_y * std::pow(x(1) - ref(1), 2);


    return cost;
  }
};
}  // namespace pole_cart
