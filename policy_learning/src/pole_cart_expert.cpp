/*!
 * @file     pole_cart_expert.cpp
 * @author   Andreas Voigt
 * @date     27.04.2021
 * @version  1.0
 * @brief    learned_expert for the pole_cart example
 */

#include "policy_learning/pole_cart_expert.h"

PoleCartExpert::PoleCartExpert(
    std::unique_ptr<Policy> policy, std::unique_ptr<Dataset> dataset):
  LearnedExpert(4, 1),
  dataset_(std::move(dataset)),
  policy_(std::move(policy))
{}

PoleCartExpert::input_t const PoleCartExpert::get_action(const observation_t& x){
  input_t action;
  policy_->forward(normalize_angle(x), action);
  return action;
}

void PoleCartExpert::save_state_action(const observation_t& x, const input_t& u){
  dataset_->cache_state_action(normalize_angle(x), u);
}

bool PoleCartExpert::collect_data(){
  return dataset_ != nullptr;
}

PoleCartExpert::input_t const PoleCartExpert::normalize_angle(input_t const& x){
   Eigen::VectorXd x_local = x;
  // Make sure that the angle is between +pi and -pi.
  // The neural net will not capture the periodicity of the angle.
  x_local(1) = fmod(x_local(1) + M_PI, 2*M_PI);
  if (x_local(1) < 0) x_local(1) += M_PI;
  else x_local(1) -= M_PI;
  return x_local;
}
