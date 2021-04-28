/*!
 * @file     panda_expert.cpp
 * @author   Andreas Voigt
 * @date     27.04.2021
 * @version  1.0
 * @brief    learned_expert for the panda example
 */

#include "policy_learning/panda_expert.h"

#include "mppi_pinocchio/model.h"

class PandaExpert::Impl{
  public:
    mppi_pinocchio::RobotModel robot_model_;
};

PandaExpert::PandaExpert(std::unique_ptr<Policy> policy, 
                         std::unique_ptr<Dataset> dataset, 
                         const std::string& robot_description):
  LearnedExpert(4, 1),
  pImpl(std::make_unique<Impl>()),
  dataset_(std::move(dataset)),
  policy_(std::move(policy))
{
  pImpl->robot_model_.init_from_xml(robot_description);
}

PandaExpert::input_t const PandaExpert::get_action(const observation_t& x){
  augmented_observation_t x_aug;
  augment_observation(x, x_aug);
  input_t u;
  policy_->forward(x_aug, u);
  return u;
}

void PandaExpert::save_state_action(const observation_t& x, const input_t& u){
  augmented_observation_t x_aug;
  augment_observation(x, x_aug);
  dataset_->cache_state_action(x_aug, u);
}

bool PandaExpert::collect_data(){
  return dataset_ != nullptr;
}

void const PandaExpert::augment_observation(observation_t const& x, 
                                            augmented_observation_t& x_aug){
  auto pose = pImpl->robot_model_.get_pose(tracked_frame_);
  x_aug.resize(x.size() + 7);
  x_aug << x, pose.translation, pose.rotation;
  // TODO: use error between pose and reference to augment state...
}


