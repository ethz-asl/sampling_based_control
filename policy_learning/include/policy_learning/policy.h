/*!
 * @file     policy.h
 * @author   Andreas Voigt
 * @date     27.04.2021
 * @version  1.0
 * @brief    Interface for NN inference
 */

#pragma once

#include <Eigen/Core>
#include <memory>

class Policy {
  public:
    virtual ~Policy(){};

  public:
    
    /**
   * @brief Does a forward pass on the policy
   * @details This function must be thread safe as it is potentially called 
   * simultaneously from multiple threads
   * @param input: input to the NN
   * @return output of the NN
   */
    virtual void const forward(const Eigen::VectorXd& input, Eigen::VectorXd& output) = 0;

    /**
   * @brief Checks that the given policy has the right sizes
   * @param input_size: size of the input
   * @param output_size: size of the output
   * @return true if sizes match
   */
    virtual bool const check_sizes(size_t input_size, size_t output_size) = 0;


};
