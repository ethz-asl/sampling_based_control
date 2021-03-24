/*!
 * @file     offline_pytorch_learner.cpp
 * @author   Andreas Voigt
 * @date     23.03.2021
 * @version  1.0
 * @brief    learned_sampler for external training with pytorch
 * @details  Writes all provided training data to a h5 file and uses a
 * torchscript file for evaluating the policy network
 */

#include "policy_learning/offline_pytorch_learner.h"
#include <iostream>

OfflinePytorchLearner::OfflinePytorchLearner(){}

OfflinePytorchLearner::~OfflinePytorchLearner(){}

OfflinePytorchLearner::input_t const OfflinePytorchLearner::get_action(
    const OfflinePytorchLearner::observation_t& x){
  input_t a;
  return a;
}

void OfflinePytorchLearner::save_state_action(
    const OfflinePytorchLearner::observation_t& x,
    const OfflinePytorchLearner::input_t& u){
  std::cout << u.transpose() << std::endl;
}


