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
#include <highfive/H5Easy.hpp>
#include <algorithm>

// Private members and functions that should not be visible in header.
class OfflinePytorchExpert::Impl{
  public:
    Impl(std::string filename):
      output_file_(filename, H5Easy::File::Overwrite)
    {};

  public:
    H5Easy::File output_file_;
    std::vector<std::vector<float>> stored_states_;
    std::vector<std::vector<float>> stored_actions_;

    bool module_initialized_ = false;
    torch::jit::script::Module module_;
};

OfflinePytorchExpert::OfflinePytorchExpert(size_t state_dim, size_t input_dim, 
    std::string filename, std::string torchscript_filename):
  LearnedExpert(state_dim, input_dim),
  pImpl(std::make_unique<Impl>(filename))
{
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
  H5Easy::dump(pImpl->output_file_, "/states", pImpl->stored_states_);
  H5Easy::dump(pImpl->output_file_, "/actions", pImpl->stored_actions_);
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
  std::vector<float> x_f, u_f;

  std::transform(x.data(), x.data()+x.size(), std::back_inserter(x_f), 
    [](double val) -> float { return (float)val; });
  std::transform(u.data(), u.data()+u.size(), std::back_inserter(u_f), 
    [](double val) -> float { return (float)val; });
  pImpl->stored_states_.push_back(x_f);
  pImpl->stored_actions_.push_back(u_f);
}
