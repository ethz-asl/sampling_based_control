/*!
 * @file     offline_pytorch_learner.h
 * @author   Andreas Voigt
 * @date     23.03.2021
 * @version  1.0
 * @brief    learned_sampler for external training with pytorch
 * @details  Writes all provided training data to a h5 file and uses a
 * torchscript file for evaluating the policy network
 */

#pragma once

#include "mppi/learned_sampler/learned_sampler.h"
#include <fstream>

class OfflinePytorchLearner: public mppi::LearnedSampler{
  public:
    using input_t = mppi::LearnedSampler::input_t;
    using observation_t = mppi::LearnedSampler::observation_t; 

  public:  
    OfflinePytorchLearner();
    ~OfflinePytorchLearner();

    input_t const get_action(const observation_t& x) override;
    void save_state_action(const observation_t& x, const input_t& u) override;
  
  private:
    std::ofstream output_file_;
    int output_precision_ = 6;
    Eigen::IOFormat cvs_format_;
};