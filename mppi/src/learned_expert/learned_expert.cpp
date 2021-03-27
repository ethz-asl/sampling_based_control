/*!
 * @file     learned_expert.cpp
 * @author   Andreas Voigt
 * @date     23.03.2021
 * @version  1.0
 * @brief    Interface for informing the trajectory sampling in MPPI by policy learning.
 */

#include "mppi/learned_expert/learned_expert.h"

namespace mppi {

LearnedExpert::LearnedExpert(size_t state_dim, size_t input_dim):
    state_dim_(state_dim),
    input_dim_(input_dim){
}

void LearnedExpert::save_rollout(const Rollout& rollout){

    for (size_t i=0; i<rollout.steps_; i++){
        save_state_action(rollout.xx[i], rollout.uu[i]);
    }
}

}