/*!
 * @file     filter_base.h
 * @author   Giuseppe Rizzi
 * @date     27.07.2020
 * @version  1.0
 * @brief    description
 */
#pragma once
#include <memory>
#include <Eigen/Core>

namespace mppi{

class FilterBase{
 public:
  using filter_ptr = std::shared_ptr<FilterBase>;
  FilterBase() = default;
  ~FilterBase() = default;

 public:
  virtual int window() = 0;
  virtual filter_ptr clone() = 0;   // virtual copy constructor
  virtual void filter(Eigen::VectorXd& channels) = 0;
};

}
