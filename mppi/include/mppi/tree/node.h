/*!
 * @file     node.h
 * @author   Etienne Walther
 * @date     08.12.2020
 * @version  1.0
 * @brief    description
 */

#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <string>

#include "mppi/utils/tree.h"

class Node {
 public:
  using node_handle = tree<Node>::iterator;

  Node() = default;
  Node(node_handle parent_node_handle, double t, double cost,
       const Eigen::VectorXd& u, const Eigen::VectorXd& x, size_t expert_type);
  ~Node() = default;

  double t_;
  Eigen::VectorXd x_;
  Eigen::VectorXd u_;
  double c_;
  double c_cum_;
  size_t expert_type_;
  node_handle parent_node_;

  std::string public_name_ = "Node_";
};
