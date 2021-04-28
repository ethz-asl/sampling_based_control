/*!
 * @file     pole_cart_expert.h
 * @author   Andreas Voigt
 * @date     27.04.2021
 * @version  1.0
 * @brief    learned_expert for the pole_cart example
 */

#pragma once

#include "mppi/learned_expert/learned_expert.h"

#include "policy_learning/dataset.h"
#include "policy_learning/policy.h"

#include <memory>
#include <string>

class PoleCartExpert: public mppi::LearnedExpert{
  public:
    using input_t = mppi::LearnedExpert::input_t;
    using observation_t = mppi::LearnedExpert::observation_t; 

  public:  
    PoleCartExpert(std::unique_ptr<Policy> policy, std::unique_ptr<Dataset> dataset);

    input_t const get_action(observation_t const& x) override;
    void save_state_action(observation_t const& x, input_t const& u) override;
    bool collect_data() override;

  private:
    observation_t const normalize_angle(observation_t const& x);

    std::unique_ptr<Dataset> dataset_ = nullptr;
    std::unique_ptr<Policy> policy_ = nullptr;
};
