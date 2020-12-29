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
#include "mppi/utils/tree.h"

class Node{
 public:
  using cost_ptr = mppi::CostBase::cost_ptr;
  using dynamics_ptr = mppi::DynamicsBase::dynamics_ptr;
  using config_t = mppi::SolverConfig;

  using node_handle = tree<Node>::iterator;

  Node() = default;
  Node(size_t step, node_handle parent_node_handle, double t, const mppi::SolverConfig& config, cost_ptr cost, const Eigen::VectorXd& u_applied, const Eigen::VectorXd& x, const Eigen::MatrixXd& sigma_inv, size_t expert_type_applied);
  ~Node() = default;

  std::string public_name_;

  node_handle parent_node_;

  double c_;
  double c_discounted;
  double c_cum_;
  double c_cum_discounted_;

	double t_;

	Eigen::VectorXd xx_;
	Eigen::VectorXd uu_applied_;

	Eigen::MatrixXd sigma_inv_;

	size_t expert_type_applied_;

 private:
  config_t config_;
  cost_ptr cost_;
};