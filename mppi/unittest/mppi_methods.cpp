/*!
 * @file     mppi_methods.cpp
 * @author   Giuseppe Rizzi
 * @date     11.06.2020
 * @version  1.0
 * @brief    description
 */

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <array>
#include <chrono>
#include "double_integrator.cpp"
#include "mppi/controller/mppi.h"
#include "mppi/sampler/gaussian_sampler.h"
#include "mppi/utils/logging.h"

using namespace Eigen;
using namespace mppi;

class PathIntegralTest : public ::testing::Test {
 protected:
  void SetUp() override {
    config.caching_factor = 0.2;
    config.rollouts = 4;
    config.lambda = 1;
    config.step_size = 0.1;
    config.horizon = 0.5;
    dynamics = std::make_shared<DoubleIntegratorDynamics>();
    cost = std::make_shared<DoubleIntegratorCost>();
    sampler = std::make_shared<mppi::GaussianSampler>(1);
    solver_ = std::make_unique<PathIntegral>(dynamics, cost, sampler, config);
  }
  SolverConfig config;
  std::unique_ptr<PathIntegral> solver_;
  std::shared_ptr<DoubleIntegratorDynamics> dynamics;
  std::shared_ptr<DoubleIntegratorCost> cost;
  mppi::GaussianSampler::sampler_ptr sampler;

  void print_rollouts() {
    for (size_t k = 0; k < solver_->config_.rollouts; k++) {
      std::cout << "Rollout " << k << ": " << std::endl;
      std::cout << solver_->rollouts_[k] << std::endl;
    }
  }

  void verbose_run(Eigen::VectorXd x0, const double t0) {
    log_info("Resetting observation.");
    solver_->set_observation(x0, t0);

    log_info("Preparing rollouts.");
    solver_->prepare_rollouts();

    log_info("Printing rollouts info.");
    print_rollouts();

    log_info("Optimal rollout");
    std::cout << solver_->get_optimal_rollout();

    log_info("Optimal rollout cached");
    std::cout << solver_->get_optimal_rollout_cache();

    log_info("Sampling trajectories");
    solver_->sample_trajectories();

    log_info("Printing rollouts info.");
    print_rollouts();

    log_info("Optimizing.");
    solver_->optimize();

    log_info("Optimal rollout");
    std::cout << solver_->get_optimal_rollout();

    log_info("Optimal rollout cached");
    std::cout << solver_->get_optimal_rollout_cache();

    log_info("Swapping policies");
    solver_->swap_policies();

    log_info("Optimal rollout");
    std::cout << solver_->get_optimal_rollout();

    log_info("Optimal rollout cached");
    std::cout << solver_->get_optimal_rollout_cache();
  }
};

TEST_F(PathIntegralTest, VerboseRun) {
  Eigen::Vector2d x0{0.0, 0.0};
  double t0 = 0.0;

  std::cout << std::string(100, '=') << std::endl;
  std::cout << " Iteraion 0 " << std::endl;
  std::cout << std::string(100, '=') << std::endl;
  verbose_run(x0, t0);

  t0 = 0.25;
  std::cout << std::string(100, '=') << std::endl;
  std::cout << " Iteraion 1 " << std::endl;
  std::cout << std::string(100, '=') << std::endl;
  verbose_run(x0, t0);

  double query_t = 0.3;
  PathIntegral::input_t u;
  log_info("Querying control input at t=" + std::to_string(query_t));
  solver_->get_input(x0, u, query_t);
  std::cout << u.transpose() << std::endl;
}

TEST_F(PathIntegralTest, InputQueries) {
  Eigen::Vector2d x0{0.0, 0.0};
  Eigen::VectorXd u = Eigen::VectorXd::Zero(1);
  double t0 = 0.0;

  config.horizon = 2.0;
  solver_ = std::make_unique<PathIntegral>(dynamics, cost, sampler, config);

  solver_->set_observation(x0, t0);
  solver_->update_policy();
  std::cout << "Optimal trajectory: " << std::endl
            << solver_->get_optimal_rollout_cache() << std::endl;

  double t;
  std::cout << "Aligned time." << std::endl;
  for (int i = 0; i < 10; i++) {
    t = t0 + i * config.step_size;
    solver_->get_input(x0, u, t);
    std::cout << "u*[" << t << "]: " << u.transpose() << std::endl;
  }

  std::cout << "Unaligned time." << std::endl;
  for (int i = 0; i < 10; i++) {
    t = t0 + 0.001 + i * config.step_size;
    solver_->get_input(x0, u, t);
    std::cout << "u*[" << t << "]: " << u.transpose() << std::endl;
  }
}

TEST_F(PathIntegralTest, AsyncRun) {
  Eigen::Vector2d x0{0.0, 0.0};
  Eigen::VectorXd u = Eigen::VectorXd::Zero(1);

  config.horizon = 2.0;
  solver_ = std::make_unique<PathIntegral>(dynamics, cost, sampler, config);

  double t0 = 0.0;
  std::atomic_bool run_once;
  run_once = false;

  solver_->set_observation(x0, t0);

  // optimization thread
  std::thread t1([&, this] {
    size_t counter = 0;
    while (counter < 5) {
      std::cout << "Update #" << counter << std::endl;
      solver_->update_policy();
      counter++;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      std::cout << "Optimal rollout: " << std::endl
                << solver_->get_optimal_rollout_cache() << std::endl;
      run_once = true;
    }
  });

  // set observation and get control thread
  std::thread t2([&, this] {
    size_t counter = 0;
    double t;
    while (counter < 5) {
      if (run_once) {
        std::cout << "Reset #" << counter << std::endl;
        t = t0 + counter * config.step_size;
        solver_->set_observation(x0, t);
        solver_->get_input(x0, u, t);
        counter++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::cout << "u*[" << t << "]: " << u.transpose() << std::endl;
      }
    }
  });

  t1.join();
  t2.join();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
