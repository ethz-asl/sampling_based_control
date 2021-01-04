/*!
 * @file     node.cpp
 * @author   Etienne Walther
 * @date     08.12.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi/tree/node.h"

#include <utility>

//Node::Node(double t, double c_parent, dynamics_ptr dynamics, const mppi::SolverConfig& config, cost_ptr cost) {
//  cost_ = cost;
//  config_ = config;
//  dynamics_ = dynamics;
//  auto current_time = std::chrono::high_resolution_clock::now();
//  timestamp_ = current_time;
//
//  public_name_ = std::to_string(current_time.time_since_epoch().count());
//
//  t_ = t;
//  c_ = c_parent + 0;
////  xx_ = dynamics_->step();
//  uu_applied_ = Eigen::VectorXd::Zero();
//  nn_applied_ = Eigen::VectorXd::Zero(1);
//}

//TODO don't do anything in node except the necessary, which is calc the statecost + cost of parent node (evt. pointer to parent node //
// (pass only observation value, and action value)

Node::Node(size_t step, node_handle parent_node_handle, double t, const mppi::SolverConfig& config, cost_ptr cost,const Eigen::VectorXd& u_applied, const Eigen::VectorXd& x, const Eigen::MatrixXd& sigma_inv, size_t expert_type_applied) {
  cost_ = std::move(cost);
  config_ = config;
  parent_node_ = parent_node_handle;

	xx_ = x;
	uu_applied_ = u_applied;

	sigma_inv_ = sigma_inv;
	expert_type_applied_ = expert_type_applied;

  t_ = t;

  auto c_cum_parent = 0;
	auto c_cum_discounted_parent = 0;
	c_ = cost_->get_stage_cost(xx_, t_);
	c_discounted = std::pow(config_.discount_factor, step) * this->c_;

  if (parent_node_ != nullptr){
		c_cum_parent = parent_node_->c_cum_;
		c_cum_discounted_parent = parent_node_->c_cum_discounted_;
  }
  c_cum_ = c_cum_parent + this->c_;
	c_cum_discounted_ = c_cum_discounted_parent + this->c_discounted;

	public_name_ = "Node_";
//	public_name_ = "Node_"+std::to_string(step)+" T: "+std::to_string(t_)+" C: "+std::to_string(c_)+" X0: "+std::to_string(xx_[0])+" U0: "+std::to_string(uu_applied_[0])+" E: "+std::to_string(expert_type_applied_);
}