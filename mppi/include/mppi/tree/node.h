/*!
 * @file     node.h
 * @author   Etienne Walther
 * @date     08.12.2020
 * @version  1.0
 * @brief    description
 */

#pragma once

#include <cstdio>
#include <string>
#include <ctime>
#include <chrono>
#include <Eigen/Dense>

#include "mppi/cost/cost_base.h"
#include "mppi/dynamics/dynamics_base.h"
#include "mppi/solver_config.h"

class Node{
 public:
  using cost_ptr = mppi::CostBase::cost_ptr;
  using dynamics_ptr = mppi::DynamicsBase::dynamics_ptr;
  using config_t = mppi::SolverConfig;

  using node_ptr = std::shared_ptr<Node>;

  Node() = default;
  Node(double t, double c_parent, dynamics_ptr dynamics, const mppi::SolverConfig& config, cost_ptr cost);
  ~Node() = default;

  std::string public_name_;
  std::chrono::high_resolution_clock::time_point timestamp_;

 private:
  double t_;
  double c_;
  Eigen::VectorXd xx_;
  Eigen::VectorXd uu_;
  Eigen::VectorXd nn_;

  size_t INPUT_DIM = 8;
  size_t STATE_DIM = 10;

  config_t config_;
  dynamics_ptr dynamics_;
  cost_ptr cost_;



//	Node * node_ptr_;
};