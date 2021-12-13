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
  double w_y = 50;
  double w_theta = 0;
  double c_leaving_field = 0;
  double x_fieldlimitpositive = 25.0;
  double x_fieldlimitnegative = -5.0;
  double y_fieldlimit = 10;
  double c_obstacle = 10000;
  double obstacle_1[3] = {14.0, 2.0, 2};

 public:
  cost_ptr create() override { return std::make_shared<PathfinderCost>(); }

  cost_ptr clone() const override {
    return std::make_shared<PathfinderCost>(*this);
  }

  cost_t compute_cost(const mppi::observation_t& x,
                      const mppi::reference_t& ref, const double t) override;
  double distance_from_obstacle_cost( double cart_position[2], double obstacle_parameters[3]);
};
}  // namespace pole_cart
