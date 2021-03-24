/*!
 * @file     learned_sampler.cpp
 * @author   Andreas Voigt
 * @date     23.03.2021
 * @version  1.0
 * @brief    Interface for informing the trajectory sampling in MPPI by policy learning.
 */

#include "mppi/learned_sampler/learned_sampler.h"
#include <stdexcept>

namespace mppi {

void LearnedSampler::save_rollout(const Rollout& rollout){

    for (size_t i=0; i<rollout.steps_; i++){
        save_state_action(rollout.xx[i], rollout.uu[i]);
    }
}

}