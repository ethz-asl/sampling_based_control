/*!
 * @file     hdf5_dataset.cpp
 * @author   Andreas Voigt
 * @date     27.04.2021
 * @version  1.0
 * @brief    Dataset in the hdf5 file format
 */

#include "policy_learning/hdf5_dataset.h"

#include <highfive/H5Easy.hpp>

class Hdf5Dataset::Impl{
  public:
    Impl(){};

  public:
    std::unique_ptr<H5Easy::File> output_file_;
    std::vector<std::vector<float>> stored_states_;
    std::vector<std::vector<float>> stored_actions_;
};

Hdf5Dataset::Hdf5Dataset(std::string const& file_path) :
  pImpl(std::make_unique<Impl>())
{
  try {
    pImpl->output_file_ = std::make_unique<H5Easy::File>
      (file_path, H5Easy::File::Overwrite);
  } catch(const H5Easy::Exception& e) {
    std::stringstream ss;
    ss << "Error while loading file: ";
    ss << file_path << ". What: ";
    ss << e.what();
    throw std::runtime_error(ss.str());
  } 
}

Hdf5Dataset::~Hdf5Dataset(){
  write();
};

void Hdf5Dataset::cache_state_action(Eigen::VectorXd const& x, Eigen::VectorXd const& u) {
  std::vector<float> x_f, u_f;
  std::transform(x.data(), x.data()+x.size(), std::back_inserter(x_f), 
    [](double val) -> float { return (float)val; });
  std::transform(u.data(), u.data()+u.size(), std::back_inserter(u_f), 
    [](double val) -> float { return (float)val; });
  pImpl->stored_states_.push_back(x_f);
  pImpl->stored_actions_.push_back(u_f);
}

void Hdf5Dataset::clear(){
  pImpl->stored_states_.clear();
  pImpl->stored_actions_.clear();
}

void Hdf5Dataset::write(){
  if (pImpl->output_file_ == nullptr){
    throw std::runtime_error("Hdf5Dataset: No file loaded.");
  }
  if(pImpl->stored_actions_.size() > 0 &&
     pImpl->stored_states_.size() > 0 ){ 
    H5Easy::dump(*pImpl->output_file_, "/states", pImpl->stored_states_);
    H5Easy::dump(*pImpl->output_file_, "/actions", pImpl->stored_actions_);
  }
}