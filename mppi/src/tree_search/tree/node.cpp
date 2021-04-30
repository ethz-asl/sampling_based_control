/*!
 * @file     node.cpp
 * @author   Etienne Walther, Giuseppe Rizzi
 * @date     08.12.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi/tree_search/tree/node.h"

Node::Node(node_handle parent_node_handle, double t, double cost,
           const Eigen::VectorXd& u, const Eigen::VectorXd& x,
           size_t expert_type)
    : t_(t),
      x_(x),
      u_(u),
      c_(cost),
      c_cum_(cost),
      expert_type_(expert_type),
      parent_node_(parent_node_handle) {
  parent_node_ = parent_node_handle;

  if (parent_node_ != nullptr) {
    c_cum_ += parent_node_->c_cum_;
  }
}