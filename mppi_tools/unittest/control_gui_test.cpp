#include "mppi_tools/control_gui.hpp"
#include "mppi/sampler/gaussian_sampler.h"
//#include <gtest/gtest.h>
//
// TEST(VisualDebuggerTest, Main) {
//  mppi_tools::VisualDebugger vd{};
//  ASSERT_TRUE(vd.init());
//  vd.render();
//}

int main(int argc, char** argv) {
  std::cout << "Starting the visual debugger test" << std::endl;
  mppi_tools::VisualDebugger vd{};
  vd.init();

  size_t n = 20;
  double counter = 0;

  mppi::SolverConfig config;
  config.rollouts = 10;
  config.input_variance = Eigen::VectorXd(10);
  config.input_variance.setConstant(0.2);

  const int steps = 100;
  const int input_dim = 4;
  const int state_dim = 4;
  std::vector<mppi::Rollout> rollouts(config.rollouts, mppi::Rollout(steps, input_dim, state_dim));
  mppi::Rollout averaged(steps, input_dim, state_dim);
  mppi::Rollout filtered(steps, input_dim, state_dim);

  std::vector<double> variance{0.01, 0.01, 0.5, 1.0};
  mppi::GaussianSampler sampler(input_dim);
  sampler.set_covariance(variance);
  static double t = 0;
  const double t_step = 0.01;

  while (true) {
    Eigen::ArrayXd weights(n);
    for (int i = 0; i < n; i++) {
      weights(i) = counter + i;
    }

    // Sample rollouts
    if (!vd.should_pause()){
      for (int i=0; i<config.rollouts; i++) {
        for (int j = 0; j < steps; j++) {
          rollouts[i].tt[j] = t + t_step * j;
          sampler.get_sample(rollouts[i].uu[j]);
        }
      }

      for (int j = 0; j < steps; j++) {
        averaged.tt[j] = t + t_step * j;
        for (const auto& roll : rollouts)
          averaged.uu[j] += roll.uu[j]/steps;
      }

      filtered.tt[0] = t;
      filtered.uu[0] = averaged.uu[0];
      for (int j = 1; j < steps; j++) {
        filtered.tt[j] = t + t_step * j;
        filtered.uu[j] = 0.8 * filtered.uu[j-1] + 0.2 * averaged.uu[j];
      }
      t += t_step;
    }

    counter += 0.0001;
    vd.reset_config(config);
    vd.reset_weights(weights);
    vd.reset_rollouts(rollouts);
    vd.reset_averaged_policy(averaged.uu);
    vd.reset_policy(filtered.uu);
    if (!vd.render()) return 0;
  }

  //  ::testing::InitGoogleTest(&argc, argv);
  //  return RUN_ALL_TESTS();
  return 0;
}
