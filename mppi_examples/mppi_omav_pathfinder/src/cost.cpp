/*!
 * @file     pendulum_cart_cost.cpp
 * @author   Giuseppe Rizzi
 * @date     10.06.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi_omav_pathfinder/cost.h"
#include <algorithm>
#include <math.h>

using namespace omav_pathfinder;

double OMAV_PathfinderCost::distance_from_obstacle_cost(double *omav_position, double *obstacle_parameters){
  double distance = std::sqrt(std::pow(*omav_position - *obstacle_parameters,2)+std::pow(*(omav_position+1) - *(obstacle_parameters+1),2));
  double distance_from_savezone = (distance - (*(obstacle_parameters+2)+1));
  double obstacle_cost = -c_obstacle*std::min(0.0, distance_from_savezone);
  return obstacle_cost;
}

mppi::CostBase::cost_t OMAV_PathfinderCost::compute_cost(const mppi::observation_t &x, const mppi::reference_t &ref, const double t){
    double cost = 0.0;
    double omav_position[3] = {x(16), x(17), x(18)};
    double obstacle_cost = distance_from_obstacle_cost(omav_position, obstacle_1);
    if (x(0) > x_fieldlimitpositive || x(0) < x_fieldlimitnegative || x(1) > y_fieldlimit || x(1) < -y_fieldlimit)
      cost += c_leaving_field;
    cost += obstacle_cost;

    //double theta = std::fmod(x(1), 2 * M_PI);
    //double delta = (theta < 0) ? ref(2) + theta : ref(2) - theta;
    //cost += w_theta * std::pow(delta, 2);

    cost += w_x * std::pow(x(16) - ref(0), 2);
    cost += w_y * std::pow(x(17) - ref(1), 2);
    cost += w_z * std::pow(x(18) - ref(2), 2);


    return cost;
}


