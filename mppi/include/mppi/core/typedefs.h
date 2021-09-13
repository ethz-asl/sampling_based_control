/*!
 * @file     typedefs.h
 * @author   Giuseppe Rizzi
 * @date     02.09.2020
 * @version  1.0
 * @brief    description
 */
#pragma once
#include <Eigen/Core>
#include <memory>
#include <vector>

namespace mppi {

class Cost;
class Dynamics;
class Solver;
class Config;
class Renderer;
class Expert;
class GaussianSampler;
class Policy;
class Filter;

typedef std::shared_ptr<Cost> cost_ptr;
typedef std::shared_ptr<Dynamics> dynamics_ptr;
typedef std::shared_ptr<Solver> solver_ptr;
typedef std::shared_ptr<GaussianSampler> sampler_ptr;
typedef std::shared_ptr<Policy> policy_ptr;
typedef Config config_t;
typedef std::shared_ptr<Renderer> renderer_ptr;
typedef std::shared_ptr<Expert> expert_ptr;
typedef std::shared_ptr<Filter> filter_ptr;

typedef Eigen::VectorXd input_t;
typedef std::vector<input_t> input_array_t;

typedef Eigen::VectorXd observation_t;
typedef std::vector<observation_t> observation_array_t;

typedef std::vector<double> time_array_t;

typedef Eigen::VectorXd reference_t;
typedef std::vector<reference_t> reference_array_t;

using cost_t = double;
using cost_vector_t = Eigen::VectorXd;

struct reference_trajectory_t {
  reference_array_t rr;
  time_array_t tt;
};

}  // namespace mppi