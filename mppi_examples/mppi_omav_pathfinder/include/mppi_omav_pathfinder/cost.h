/*!
 * @file     cost.h
 * @author   Matthias Studiger
 * @date     14.03.2021
 * @version  1.0
 * @brief    description
 */

#pragma once
#include <math.h>
#include <mppi/cost/cost_base.h>

namespace omav_pathfinder {

class OMAV_PathfinderCost : public mppi::CostBase {
 public:
  OMAV_PathfinderCost() = default;
  ~OMAV_PathfinderCost() = default;

 private:
  // TODO: Change that these values are read form a parameter file and update values
  double w_x = 10;
  double w_y = 10;
  double w_z = 10;
  double w_theta = 0;
  double c_leafing_field = 0;
  double x_fieldlimitpositive = 25.0;
  double x_fieldlimitnegative = -5.0;
  double y_fieldlimit = 10;
  double c_obstacle = 10000;
  double obstacle_1[3] = {14.0, 2.0, 2};

 public:
  cost_ptr create() override { return std::make_shared<OMAV_PathfinderCost>(); }

  cost_ptr clone() const override {
    return std::make_shared<OMAV_PathfinderCost>(*this);
  }

  cost_t compute_cost(const mppi::observation_t& x,
                      const mppi::reference_t& ref, const double t) override;
  // TODO: Update to 3D space formulation!
  double distance_from_obstacle_cost( double cart_position[2], double obstacle_parameters[3]);
};
}  // namespace omav_pathfinder
