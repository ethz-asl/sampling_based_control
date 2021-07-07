/*!
 * @file     torch_script_policy.h
 * @author   Andreas Voigt
 * @date     27.04.2021
 * @version  1.0
 * @brief    Interface for NN inference with torch script
 */

#pragma once

#include "policy_learning/policy.h"

#include <string>

class TorchScriptPolicy: public Policy {
  public:

  TorchScriptPolicy(std::string const& model_path);
  ~TorchScriptPolicy();

  TorchScriptPolicy(TorchScriptPolicy&&) = default;
  TorchScriptPolicy& operator=(TorchScriptPolicy&&) = default;
  TorchScriptPolicy& operator=(TorchScriptPolicy const& other);

  void const forward(const Eigen::VectorXd& input, Eigen::VectorXd& output) override;
  bool const check_sizes(size_t input_size, size_t output_size) override;

  private:
    // Use pImpl idom as described here: https://arne-mertz.de/2019/01/the-pimpl-idiom/
    // the goal is to hide implementation details (such as torch) from this header file
    // so that packages that include this header don't need the internal dependencies.
    class Impl;
    std::unique_ptr<Impl> pImpl;
  
};
    