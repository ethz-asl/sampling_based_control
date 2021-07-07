/*!
 * @file     dataset.h
 * @author   Andreas Voigt
 * @date     27.04.2021
 * @version  1.0
 * @brief    Interface for saving data to a dataset
 */

#pragma once

#include <Eigen/Core>
#include <memory>

class Dataset {
  public:
    virtual ~Dataset(){};

  public:
    /**
   * @brief Writes a state-action pair to the cache
   * @param x: state
   * @param u: action
   */
    virtual void cache_state_action(Eigen::VectorXd const& x, Eigen::VectorXd const& u) = 0;

    /**
   * @brief Clears all cached data
   */
    virtual void clear() = 0;

    /**
   * @brief Writes cached data to file
   */
    virtual void write() = 0;
};
