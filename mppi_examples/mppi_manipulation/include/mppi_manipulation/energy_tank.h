//
// Created by giuseppe on 11.08.21.
//

#pragma once

namespace manipulation {

class EnergyTank {
 public:
  EnergyTank() = default;
  ~EnergyTank() = default;

 public:
  void step(double e, double dt);
  void reset(double x, double t);
  inline const double& get_state() { return tank_state_; }

 private:
  double tank_state_;
  double tank_energy_;
  double integration_delta_;
};

}  // namespace manipulation