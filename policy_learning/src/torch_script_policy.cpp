/*!
 * @file     torch_script_policy.h
 * @author   Andreas Voigt
 * @date     27.04.2021
 * @version  1.0
 * @brief    Interface for NN inference with torch script
 */

#include "policy_learning/torch_script_policy.h"

#include <torch/script.h>

class TorchScriptPolicy::Impl{
  public:
    torch::jit::script::Module module_;
};

TorchScriptPolicy::TorchScriptPolicy(std::string const& model_path):
  pImpl(std::make_unique<Impl>())
{
  try {
    pImpl->module_ = torch::jit::load(model_path);
  } catch (const c10::Error& e) {
    std::stringstream error_msg;
    error_msg << "Could not load torch module: " << model_path;
    throw std::runtime_error(error_msg.str());
  }
}

// Note that we need to put the default desctructor in the implementation
// file (.cpp). If we put it in the header, it would be generated inline
// where the type of Impl would still be incomplete -> compile error.
TorchScriptPolicy::~TorchScriptPolicy() = default;

bool const TorchScriptPolicy::check_sizes(size_t input_size, size_t output_size){
  // Check that one can do a forward pass and that the dimensions match.
  bool ok = true;
  std::stringstream error_msg;
  std::vector<torch::jit::IValue> test_state;
  test_state.push_back(torch::rand(input_size));
  try {
    torch::Tensor action = pImpl->module_.forward(test_state).toTensor();
    if(action.dim() != 1 || (size_t)action.size(0) != output_size) {
      error_msg << "Torch model returns vector of size: " << action.sizes() 
      << " expected: [" << output_size << "]";
      ok = false;
    }
  } catch (const c10::Error& e) {
    ok = false;
    error_msg << "Forward propagation in torch module failed. "
    << "Are the state and NN-input dimensions the same?";
  }
  return ok;
}

void const TorchScriptPolicy::forward(const Eigen::VectorXd& input, Eigen::VectorXd& output) {
  Eigen::VectorXf input_f = input.cast<float>(); 
  auto options = torch::TensorOptions().dtype(torch::kFloat32);
  at::Tensor tmp = torch::from_blob(input_f.data(), {input_f.size()}, options);
  at::Tensor t_action = pImpl->module_.forward({tmp}).toTensor();
  // This should also copy the values (due to the assignment). 
  // Using the t_action's data directly is not safe because eigen doesn't
  // take the ownership and the memory could get deallocated.
  Eigen::VectorXf output_t = Eigen::Map<Eigen::VectorXf>(
    t_action.data_ptr<float>(), t_action.size(0));
  output = output_t.cast<double>(); 
}

TorchScriptPolicy& TorchScriptPolicy::operator=(TorchScriptPolicy const& other){
    *pImpl = *other.pImpl;
    return *this;
}
