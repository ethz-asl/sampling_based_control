//
// Created by giuseppe on 12/16/20.
//

#pragma once

#include "mppi_est/typedefs.hpp"

namespace mppi_est{

class Model{
 public:
  Model() = default;
  ~Model() = default;

 public:
  virtual std::unique_ptr<Model> create() = 0;
  virtual void step(const transition_tuple_t& z) = 0;
};
}