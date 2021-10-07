/*!
 * @file     rollout.h
 * @author   Giuseppe Rizzi
 * @date     01.07.2020
 * @version  1.0
 * @brief    description
 */
#pragma once
#include <Eigen/Core>
#include <vector>

namespace mppi {

/// Structur to collect the rollout information
struct Rollout {
  Rollout();
  explicit Rollout(size_t steps, size_t input_dim, size_t state_dim);

  bool valid = true;
  size_t steps_;
  size_t input_dim_;
  size_t state_dim_;
  double total_cost = 0.0;
  
  Eigen::VectorXd cc;
  std::vector<double> tt;
  std::vector<Eigen::VectorXd> uu;
  std::vector<Eigen::VectorXd> nn;
  std::vector<Eigen::VectorXd> xx;

  void clear();
  void clear_cost();
  void clear_input();
  void clear_observation();

  bool operator<(const Rollout& roll) const;
};

}  // namespace mppi

std::ostream& operator<<(std::ostream& os, const mppi::Rollout& r);