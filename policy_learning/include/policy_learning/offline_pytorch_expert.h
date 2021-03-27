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
#include <fstream>

class OfflinePytorchExpert: public mppi::LearnedExpert{
  public:
    using input_t = mppi::LearnedExpert::input_t;
    using observation_t = mppi::LearnedExpert::observation_t; 

  public:  
    OfflinePytorchExpert();
    ~OfflinePytorchExpert();

    input_t const get_action(const observation_t& x) override;
    void save_state_action(const observation_t& x, const input_t& u) override;
  
  private:
    std::ofstream output_file_;
    int output_precision_ = 6;
    Eigen::IOFormat cvs_format_;
};