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
    bool log_data_ = true;

    std::vector<std::vector<float>> stored_states_;
    std::vector<std::vector<float>> stored_actions_;

    std::unique_ptr<H5Easy::File> output_file_= nullptr;
    std::unique_ptr<torch::jit::script::Module> module_= nullptr;
};

OfflinePytorchExpert::OfflinePytorchExpert(
    size_t state_dim, size_t input_dim):
  LearnedExpert(state_dim, input_dim),
  pImpl(std::make_unique<Impl>())
{}

OfflinePytorchExpert::~OfflinePytorchExpert(){
  dump_cache();  
}

void const OfflinePytorchExpert::dump_cache(){
  if(pImpl->output_file_){
    H5Easy::dump(*pImpl->output_file_, "/states", pImpl->stored_states_);
    H5Easy::dump(*pImpl->output_file_, "/actions", pImpl->stored_actions_);
  }
}

void OfflinePytorchExpert::set_data_logging(bool enable){
  pImpl->log_data_ = enable;
}

void OfflinePytorchExpert::clear_data_cache(){
  pImpl->stored_states_.clear();
  pImpl->stored_actions_.clear();
}

void OfflinePytorchExpert::set_output_path(std::string path){
  pImpl->output_file_ = std::make_unique<H5Easy::File>(path, H5Easy::File::Overwrite);
}

void OfflinePytorchExpert::load_torch_module(std::string path){
  try {
    pImpl->module_ = std::make_unique<torch::jit::script::Module>
      (torch::jit::load(path));
  } catch (const c10::Error& e) {
    std::stringstream error_msg;
    error_msg << "Could not load torch module: " << path;
    throw std::runtime_error(error_msg.str());
  }
  // Check that one can do a forward pass and that the dimensions match.
  bool ok = true;
  std::stringstream error_msg;
  std::vector<torch::jit::IValue> test_state;
  test_state.push_back(torch::rand(state_dim_));
  try {
    torch::Tensor action = pImpl->module_->forward(test_state).toTensor();
    if(action.dim() != 1 || (size_t)action.size(0) != input_dim_) {
      error_msg << "Torch model returns vector of size: " << action.sizes() 
      << " expected: [" << input_dim_ << "]";
      ok = false;
    }
  } catch (const c10::Error& e) {
    ok = false;
    error_msg << "Forward propagation in torch module failed. "
    << "Are the state and NN-input dimensions the same?";
  }
  if (!ok) {
    throw std::length_error(error_msg.str());
    pImpl->module_ = nullptr;
  }
}

OfflinePytorchExpert::input_t const OfflinePytorchExpert::get_action(
    const OfflinePytorchExpert::observation_t& x){
  input_t action;
  if(pImpl->module_){
    Eigen::VectorXf x_f = x.cast<float>(); 
    auto options = torch::TensorOptions().dtype(torch::kFloat32);
    at::Tensor tmp = torch::from_blob(x_f.data(), {x_f.size()}, options);
    std::vector<torch::jit::IValue> t_observation;
    t_observation.push_back(tmp);
    at::Tensor t_action = pImpl->module_->forward(t_observation).toTensor();
    // This should also copy the values (due to the assignment). 
    // Using the t_action's data directly is not safe because eigen doesn't
    // take the ownership and the memory could get deallocated.
    Eigen::VectorXf action_f = Eigen::Map<Eigen::VectorXf>(
      t_action.data_ptr<float>(), t_action.size(0));
    action = action_f.cast<input_t::Scalar>();
  } else {
    // TODO(ANDY): this is really not ideal because nobody notices that no module was loaded
    // at some point we should require to load a module
    action = input_t::Zero(input_dim_);
  }
  return action;
}

void OfflinePytorchExpert::save_state_action(
    const OfflinePytorchExpert::observation_t& x,
    const OfflinePytorchExpert::input_t& u){
  if (pImpl->log_data_){
    std::vector<float> x_f, u_f;
    std::transform(x.data(), x.data()+x.size(), std::back_inserter(x_f), 
      [](double val) -> float { return (float)val; });
    std::transform(u.data(), u.data()+u.size(), std::back_inserter(u_f), 
      [](double val) -> float { return (float)val; });
    pImpl->stored_states_.push_back(x_f);
    pImpl->stored_actions_.push_back(u_f);
  }
}

bool OfflinePytorchExpert::collect_data(){
  return pImpl->log_data_;
}
