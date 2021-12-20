/*!
 * @file     rollout.cpp
 * @author   Giuseppe Rizzi
 * @date     12.08.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi/controller/rollout.h"

using namespace mppi;

Rollout::Rollout() {}

// Copy constructor:
Rollout::Rollout(const Rollout& r) {
  steps_ = r.steps_;
  input_dim_ = r.input_dim_;
  state_dim_ = r.state_dim_;
  total_cost = r.total_cost;
  cc = r.cc;
  tt = r.tt;
  uu = r.uu;
  nn = r.nn;
  xx = r.xx;
}

Rollout::Rollout(size_t steps, size_t input_dim, size_t state_dim) {
  steps_ = steps;
  input_dim_ = input_dim;
  state_dim_ = state_dim;
  cc = Eigen::VectorXd::Zero(steps_);
  tt.resize(steps_, 0.0);
  uu.resize(steps_, Eigen::VectorXd::Zero(input_dim_));
  nn.resize(steps_, Eigen::VectorXd::Zero(input_dim_));
  xx.resize(steps_, Eigen::VectorXd::Zero(state_dim_));
}

void Rollout::clear() {
  total_cost = 0.0;
  tt.resize(steps_, 0.0);
  cc.setZero();
  uu.clear();
  uu.resize(steps_, Eigen::VectorXd::Zero(input_dim_));
  nn.clear();
  nn.resize(steps_, Eigen::VectorXd::Zero(input_dim_));
  xx.clear();
  xx.resize(steps_, Eigen::VectorXd::Zero(state_dim_));
}

void Rollout::clear_cost() {
  total_cost = 0.0;
  cc.setZero();
}

void Rollout::clear_input() {
  uu.clear();
  uu.resize(steps_, Eigen::VectorXd::Zero(input_dim_));
}

void Rollout::clear_observation() {
  xx.clear();
  xx.resize(steps_, Eigen::VectorXd::Zero(state_dim_));
}

bool Rollout::operator<(const Rollout& roll) const {
  return (total_cost < roll.total_cost);
}

std::ostream& operator<<(std::ostream& os, const mppi::Rollout& r) {
  os << "[t]: ";
  for (const auto& t : r.tt) os << t << ", ";
  os << std::endl;

  os << "[x]: ";
  for (const auto& x : r.xx) os << "(" << x.transpose() << ") ";
  os << std::endl;

  os << "[u]: ";
  for (const auto& u : r.uu) os << "(" << u.transpose() << ") ";
  os << std::endl;

  os << "[e]: ";
  for (const auto& e : r.nn) os << "(" << e.transpose() << ") ";
  os << std::endl;
  os << "Total cost: " << r.total_cost << std::endl;
}
