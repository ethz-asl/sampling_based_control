/*!
 * @file     pendulum_cart_cost.cpp
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_pathfinder/cost.h"
#include <math.h>
#include <algorithm>

using namespace pathfinder;

double PathfinderCost::distance_from_obstacle_cost(double *cart_position, double *obstacle_parameters){
  double distance = std::sqrt(std::pow(*cart_position - *obstacle_parameters,2)+std::pow(*(cart_position+1) - *(obstacle_parameters+1),2));
  double distance_from_savezone = (distance - (*(obstacle_parameters+2)+1));
  double obstacle_cost = -c_obstacle*std::min(0.0, distance_from_savezone);
  return obstacle_cost;
}

mppi::CostBase::cost_t PathfinderCost::compute_cost(const mppi::observation_t &x, const mppi::reference_t &ref, const double t){
    double cost = 0.0;
    double cart_position[2] = {x(0), x(1)};
    double obstacle_cost = distance_from_obstacle_cost(cart_position, obstacle_1);
    if (x(0) > x_fieldlimitpositive || x(0) < x_fieldlimitnegative || x(1) > y_fieldlimit || x(1) < -y_fieldlimit)
      cost += c_leafing_field;
    cost += obstacle_cost;

    double theta = std::fmod(x(1), 2 * M_PI);
    double delta = (theta < 0) ? ref(2) + theta : ref(2) - theta;
    cost += w_theta * std::pow(delta, 2);

    cost += w_x * std::pow(x(0) - ref(0), 2);
    cost += w_y * std::pow(x(1) - ref(1), 2);


    return cost;
}

