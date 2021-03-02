/*!
 * @file     typedefs.h
 * @author   Giuseppe Rizzi
 * @date     02.09.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

namespace mppi {

typedef Eigen::VectorXd input_t;
typedef std::vector<input_t> input_array_t;

typedef Eigen::VectorXd observation_t;
typedef std::vector<observation_t> observation_array_t;

typedef std::vector<double> time_array_t;

typedef Eigen::VectorXd reference_t;
typedef std::vector<reference_t> reference_array_t;

struct reference_trajectory_t {
  reference_array_t rr;
  time_array_t tt;
};

}  // namespace mppi