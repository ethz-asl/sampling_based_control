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
#include <torch/script.h>
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

    bool module_initialized_ = false;
    torch::jit::script::Module module_;
};

OfflinePytorchExpert::OfflinePytorchExpert(size_t state_dim, size_t input_dim, 
    std::string filename, std::string torchscript_filename):
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

  try {
    pImpl->module_ = torch::jit::load(torchscript_filename);
    // Check that one can do a forward pass and that the dimensions match.
    std::vector<torch::jit::IValue> test_state;
    test_state.push_back(torch::rand(state_dim_));
    torch::Tensor action = pImpl->module_.forward(test_state).toTensor();
    if(action.dim() == 1 && (size_t)action.size(0) == input_dim_){
      pImpl->module_initialized_ = true;
    } else {
      std::stringstream error_msg;
      error_msg << "Torch model returns vector of size: " << action.sizes() 
        << " expected: [" << input_dim_ << "]";
      throw std::length_error(error_msg.str());
    }
  }
  catch (const c10::Error& e) {
    std::cerr << "error loading the model: " 
      << torchscript_filename << std::endl;
  }
}

OfflinePytorchExpert::~OfflinePytorchExpert(){
  pImpl->output_file_.close();
}

OfflinePytorchExpert::input_t const OfflinePytorchExpert::get_action(
    const OfflinePytorchExpert::observation_t& x){
  input_t action;
  // TODO(ANDY): this here is only temporary for testing purposes.
  // We always need to load a module later on.
  if(pImpl->module_initialized_){
    // we need to copy the observation unfortunately because x is const and 
    // from_blob doesn't seem to support const pointers.
    auto x_copy = x; 
    auto options = torch::TensorOptions().dtype(torch::kFloat32);
    at::Tensor tmp = torch::from_blob(x_copy.data(), {x.size()}, options);
    std::vector<torch::jit::IValue> t_observation;
    t_observation.push_back(tmp);
    at::Tensor t_action = pImpl->module_.forward(t_observation).toTensor();
    // This should also copy the values (due to the assignment). 
    // Using the t_action's data directly is not safe because eigen doesn't
    // take the ownership and the memory could get deallocated.
    Eigen::VectorXf action_f = Eigen::Map<Eigen::VectorXf>(
      t_action.data_ptr<float>(), t_action.size(0));
    action = action_f.cast<input_t::Scalar>();
  }
  
  return action;
}

void OfflinePytorchExpert::save_state_action(
    const OfflinePytorchExpert::observation_t& x,
    const OfflinePytorchExpert::input_t& u){
  // TODO(Andy): We should use a more sophisticated format at one point... 
  pImpl->output_file_ << x.format(pImpl->cvs_format_) << "," 
    << u.format(pImpl->cvs_format_) << std::endl;
}
