//
// Created by giuseppe on 11.08.21.
//

#include <cmath>
#include "mppi_manipulation/energy_tank.h"

using namespace manipulation;

void EnergyTank::step(double e, double dt) {
  tank_energy_ += dt * e;
  tank_energy_ = std::max(
      0.0,
      tank_energy_ + dt * e);  // should not kick in if const always satisfied
  tank_state_ = std::sqrt(2.0 * tank_energy_);
}

void EnergyTank::reset(double x, double t){
  tank_state_ = x;
  tank_energy_ = 0.5 * x * x;
}
