/*!
 * @file     offline_pytorch_expert.cpp
 * @author   Andreas Voigt
 * @date     23.03.2021
 * @version  1.0
 * @brief    learned_expert for external training with pytorch
 * @details  Writes all provided training data to a h5 file and uses a
 * torchscript file for evaluating the policy network
 */

#include "policy_learning/offline_pytorch_expert.h"
#include <iostream>
#include <stdexcept>
#include <torch/torch.h>
#include <fstream>

// Private members and functions that should not be visible in header.
class OfflinePytorchExpert::Impl{
  public:
    Impl(std::string filename):
      output_file_(filename),
      cvs_format_(output_precision_, Eigen::DontAlignCols, ",", ",")
    {};

  public:
    std::ofstream output_file_;
    int output_precision_ = 6;
    Eigen::IOFormat cvs_format_;
};

OfflinePytorchExpert::OfflinePytorchExpert(size_t state_dim, size_t input_dim, 
    std::string filename):
  LearnedExpert(state_dim, input_dim),
  pImpl(std::make_unique<Impl>(filename))
{
  if(pImpl->output_file_.is_open()){
    for(size_t i=0; i<state_dim_; i++){
      pImpl->output_file_ << "x" << i << ",";
    }
    for(size_t i=0; i<input_dim_-1; i++){
      pImpl->output_file_ << "u" << i << ",";
    }
    pImpl->output_file_ << "u" << input_dim_-1 << std::endl;
  } else {
    throw std::invalid_argument("Could not open file " + filename);
  }

  torch::Tensor tensor = torch::rand({2, 3});
  std::cout << "*******LIB_TORCH TEST:********" << std::endl;
  std::cout << tensor << std::endl;
}

OfflinePytorchExpert::~OfflinePytorchExpert(){
  pImpl->output_file_.close();
}

OfflinePytorchExpert::input_t const OfflinePytorchExpert::get_action(
    const OfflinePytorchExpert::observation_t& x){
  input_t a;
  return a;
}

void OfflinePytorchExpert::save_state_action(
    const OfflinePytorchExpert::observation_t& x,
    const OfflinePytorchExpert::input_t& u){
  // TODO(Andy): We should use a more sophisticated format at one point... 
  pImpl->output_file_ << x.format(pImpl->cvs_format_) << "," 
    << u.format(pImpl->cvs_format_) << std::endl;
}
