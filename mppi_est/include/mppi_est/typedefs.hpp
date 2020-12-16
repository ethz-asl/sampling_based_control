//
// Created by giuseppe on 12/16/20.
//

#pragma once
#include <Eigen/Core>

namespace mppi_est{

using vector_t = Eigen::VectorXd;
using matrix_t = Eigen::MatrixXd;

struct transition_tuple_t {
  vector_t x;
  vector_t u;
  mutable vector_t x_next;
};


}