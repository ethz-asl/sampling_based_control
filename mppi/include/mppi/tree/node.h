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
  Node(size_t step, node_ptr parent_node, double t, const mppi::SolverConfig& config, cost_ptr cost, Eigen::VectorXd u, Eigen::VectorXd x, Eigen::MatrixXd sigma, Eigen::MatrixXd sigma_inv);
  ~Node() = default;

  std::string public_name_;
  std::chrono::high_resolution_clock::time_point timestamp_;

  node_ptr parent_node_;

  double c_;
  double c_discounted;
  double c_cum_;
  double c_cum_discounted_;

	double t_;

	Eigen::VectorXd xx_;
	Eigen::VectorXd uu_;
	Eigen::VectorXd nn_;

	Eigen::MatrixXd sigma_;
	Eigen::MatrixXd sigma_inv_;

 private:
  config_t config_;
  cost_ptr cost_;
};