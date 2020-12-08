/*!
 * @file     node.cpp
 * @author   Etienne Walther
 * @date     08.12.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi/tree/node.h"

Node::Node(double t, double c_parent, dynamics_ptr dynamics, const mppi::SolverConfig& config, cost_ptr cost) {
  cost_ = cost;
  config_ = config;
  dynamics_ = dynamics;
  auto current_time = std::chrono::high_resolution_clock::now();
  timestamp_ = current_time;

  public_name_ = std::to_string(current_time.time_since_epoch().count());

  t_ = t;
  c_ = c_parent + 0;
  xx_ = dynamics_->step();
  uu_ = Eigen::VectorXd::Zero(INPUT_DIM);
  nn_ = Eigen::VectorXd::Zero(INPUT_DIM);
}

//TODO don't do anything in node except the necessary, which is calc the statecost + cost of parent node (evt. pointer to parent node //
// (pass only observation value, and action value)
Node::Node(node_ptr parent_node, double t, mppi::observation_t observation, const mppi::SolverConfig& config, cost_ptr cost) {
  cost_ = cost;
  config_ = config;
  timestamp_ = std::chrono::high_resolution_clock::now();;
  public_name_ = std::to_string(timestamp_.time_since_epoch().count());

  t_ = t;
  c_ = parent_node->c_ + cost_->get_stage_cost();
  //xx_ =
  uu_ = Eigen::VectorXd::Zero(INPUT_DIM);
  nn_ = Eigen::VectorXd::Zero(INPUT_DIM);
}
