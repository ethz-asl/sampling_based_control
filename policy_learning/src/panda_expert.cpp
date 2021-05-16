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
  LearnedExpert(7, 7),
  pImpl(std::make_unique<Impl>()),
  dataset_(std::move(dataset)),
  policy_(std::move(policy))
{
  pImpl->robot_model_.init_from_xml(robot_description);
  if (policy_) policy_->check_sizes(13, 7);
}

PandaExpert::~PandaExpert() = default;

PandaExpert& PandaExpert::operator=(PandaExpert const& other){
  *pImpl = *other.pImpl;
  return *this;
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
  pImpl->robot_model_.update_state(x.head<7>());

  if (timed_ref_.rr.size() == 0){
    throw std::runtime_error("PandaExpert: No reference set!");
  }

  Eigen::Vector3d ref_t = timed_ref_.rr[0].head<3>();
  Eigen::Quaterniond ref_q(timed_ref_.rr[0].segment<4>(3));
  Eigen::Matrix<double, 6, 1> error;
  pImpl->robot_model_.get_error(tracked_frame_, ref_q, ref_t, error);

  Eigen::Vector3d obstacle_position = timed_ref_.rr[0].tail<3>();

  x_aug.resize(7 + 6);
  x_aug << x.head<7>(), error; 
}
