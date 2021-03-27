/*!
 * @file     offline_pytorch_expert.h
 * @author   Andreas Voigt
 * @date     23.03.2021
 * @version  1.0
 * @brief    learned_expert for external training with pytorch
 * @details  Writes all provided training data to a h5 file and uses a
 * torchscript file for evaluating the policy network
 */

#pragma once

#include "mppi/learned_expert/learned_expert.h"
#include <memory>

class OfflinePytorchExpert: public mppi::LearnedExpert{
  public:
    using input_t = mppi::LearnedExpert::input_t;
    using observation_t = mppi::LearnedExpert::observation_t; 

  public:  
    explicit OfflinePytorchExpert(size_t state_dim, size_t input_dim, 
                                  std::string filename);
    ~OfflinePytorchExpert();
    OfflinePytorchExpert(OfflinePytorchExpert&&) = default;
    OfflinePytorchExpert& operator=(OfflinePytorchExpert&&) = default;
    // Delete copy constructor because we would otherwise write on the same object.
    OfflinePytorchExpert& operator=(OfflinePytorchExpert const& other) = delete;


    input_t const get_action(const observation_t& x) override;
    void save_state_action(const observation_t& x, const input_t& u) override;
  
  private:
    // Use pImpl idom as described here: https://arne-mertz.de/2019/01/the-pimpl-idiom/
    // the goal is to hide implementation details (such as torch) from this header file
    // so that packages that include this header don't need the internal dependencies.
    class Impl;
    std::unique_ptr<Impl> pImpl;
};