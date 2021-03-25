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

OfflinePytorchLearner::OfflinePytorchLearner():
  output_file_("/home/andreas/out.cvs"),
  cvs_format_(output_precision_, Eigen::DontAlignCols, ",", ",")
  {}

OfflinePytorchLearner::~OfflinePytorchLearner(){
  output_file_.close();
}

OfflinePytorchLearner::input_t const OfflinePytorchLearner::get_action(
    const OfflinePytorchLearner::observation_t& x){
  input_t a;
  return a;
}

void OfflinePytorchLearner::save_state_action(
    const OfflinePytorchLearner::observation_t& x,
    const OfflinePytorchLearner::input_t& u){
  // TODO(Andy): We should use a more sophisticated format at one point...
  if (output_file_) {
    output_file_ << x.size() << "," << u.size() << ","
      << x.format(cvs_format_) << u.format(cvs_format_) << std::endl;
  }
  else std::cout << "Error occured in outstream" << std::endl;
}
