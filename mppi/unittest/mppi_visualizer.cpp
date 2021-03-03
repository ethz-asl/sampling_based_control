/*!
 * @file     mppi_visualizer.cpp
 * @author   Giuseppe Rizzi
 * @date     23.07.2020
 * @version  1.0
 * @brief    description
 */

#include "double_integrator.cpp"
#include "mppi/sampler/gaussian_sampler.h"
#include "mppi/visualization/path_integral_visualizer.h"

class DoubleIntegratorPIDebugTest : public PathIntegralVisualizer {
 public:
  DoubleIntegratorPIDebugTest(DynamicsBase::dynamics_ptr dynamics,
                              CostBase::cost_ptr cost,
                              GaussianSampler::sampler_ptr sampler,
                              const SolverConfig& config)
      : PathIntegralVisualizer(dynamics, cost, sampler, config){};

  void visualize_single_trajectory(const observation_array_t& traj,
                                   double /*dt*/) override {
    std::cout << "Visualizing trajectory." << std::endl;
  }

  void visualize_optimal_trajectory(const observation_array_t& traj) override {
    std::cout << "Visualizing optimal trajectory." << std::endl;
  }
};

int main(int argc, char** argv) {
  SolverConfig config;
  config.caching_factor = 0.2;
  config.rollouts = 5;
  config.lambda = 1;
  config.step_size = 0.1;
  config.horizon = 0.3;
  config.debug_print = false;
  config.verbose = true;

  auto dynamics = std::make_shared<DoubleIntegratorDynamics>();
  auto cost = std::make_shared<DoubleIntegratorCost>();
  auto sampler = std::make_shared<mppi::GaussianSampler>(1);

  DoubleIntegratorPIDebugTest di_pi_debugger(dynamics, cost, sampler, config);
  std::cout << "Here" << std::endl;
  int iterations = 5;
  DoubleIntegratorPIDebugTest::input_t u;
  DoubleIntegratorPIDebugTest::observation_t x;
  x = DoubleIntegratorPIDebugTest::observation_t::Zero(2);
  mppi::state_input_pair xu;
  xu.x = x;
  xu.u = u;
  for (size_t i = 0; i < iterations; i++) {
    xu = di_pi_debugger.run(xu.x, 0.0, true);
  }
  return 0;
}