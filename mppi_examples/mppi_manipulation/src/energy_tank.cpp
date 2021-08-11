//
// Created by giuseppe on 11.08.21.
//

#include <cmath>
#include "mppi_manipulation/energy_tank.h"

using namespace manipulation;

// TODO (giuseppe verify these equations)
void EnergyTank::step(double e, double dt) {
  double delta = std::pow(e * dt / (std::sqrt(2) * tank_state_), 2);
  integration_delta_ += delta;
  tank_energy_ += dt * e + delta;
  tank_state_ += dt * e;
}

void EnergyTank::reset(double x, double t){
  tank_state_ = x;
  tank_energy_ = 0.5 * x * x;
}
