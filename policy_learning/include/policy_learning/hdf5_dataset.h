/*!
 * @file     hdf5_dataset.h
 * @author   Andreas Voigt
 * @date     27.04.2021
 * @version  1.0
 * @brief    Dataset in the hdf5 file format
 */

#pragma once

#include "policy_learning/dataset.h"

#include <string>

class Hdf5Dataset: public Dataset {
  public:
    Hdf5Dataset(std::string const& file_path);
    ~Hdf5Dataset();

    Hdf5Dataset(Hdf5Dataset&&) = default;
    Hdf5Dataset& operator=(Hdf5Dataset&&) = default;
    // We don't want to copy a dataset, otherwise two instances might write the same file
    Hdf5Dataset& operator=(Hdf5Dataset const& other) = delete;

    void cache_state_action(Eigen::VectorXd const& x, Eigen::VectorXd const& u) override;
    void clear() override;
    void write() override;

  private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
  
};
    