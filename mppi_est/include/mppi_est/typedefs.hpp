//
// Created by giuseppe on 12/16/20.
//

#pragma once
#include <Eigen/Core>
#include <iostream>

#define PRINT_ERROR(msg) std::cout << "\033[1;31m[ERROR] : " << msg + "\033[0m" << std::endl

namespace mppi_est {

using vector_t = Eigen::VectorXd;
using matrix_t = Eigen::MatrixXd;

struct transition_tuple_t {
  vector_t x;
  vector_t u;
  mutable vector_t x_next;

  void print() const {
    std::cout << "x: " << x.transpose() << std::endl
              << "x_next: " << x_next.transpose() << std::endl
              << "u: " << u.transpose() << std::endl;
  }
};

}  // namespace mppi_est
