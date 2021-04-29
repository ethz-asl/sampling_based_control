/*!
 * @file     panda_expert.h
 * @author   Andreas Voigt
 * @date     28.04.2021
 * @version  1.0
 * @brief    learned_expert for the panda example
 */

#pragma once

#include "mppi/learned_expert/learned_expert.h"

#include "policy_learning/dataset.h"
#include "policy_learning/policy.h"

#include <memory>
#include <string>

class PandaExpert: public mppi::LearnedExpert{
  public:
    using input_t = mppi::LearnedExpert::input_t;
    using observation_t = mppi::LearnedExpert::observation_t; 
    using augmented_observation_t = mppi::LearnedExpert::observation_t;

  public:  
    PandaExpert(std::unique_ptr<Policy> policy, std::unique_ptr<Dataset> dataset, 
                const std::string& robot_description);
    ~PandaExpert();
    PandaExpert(PandaExpert&&) = default;
    PandaExpert& operator=(PandaExpert&&) = default;
    PandaExpert& operator=(PandaExpert const& other);

    input_t const get_action(observation_t const& x) override;
    void save_state_action(observation_t const& x, input_t const& u) override;
    bool collect_data() override;

  private:
    void const augment_observation(observation_t const& x, 
                                   augmented_observation_t& x_aug);

  private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    std::string tracked_frame_ = "panda_hand";
    std::unique_ptr<Dataset> dataset_ = nullptr;
    std::unique_ptr<Policy> policy_ = nullptr;
};
