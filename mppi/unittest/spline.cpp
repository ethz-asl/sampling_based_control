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
#include "mppi/policies/receding_horizon_spline.h"

using namespace Eigen;
using namespace mppi;

class RecedingHorizonSplineTest : public ::testing::Test {
 protected:
  void SetUp() override {
    config_.samples = 5;
    config_.step_size = 0.1;
    config_.sigma = 1.0;
    config_.min_value = -1.0;
    config_.max_value = 1.0;
    config_.dt = 0.01;
    config_.cp_dt = 0.1;
    config_.horizon = 1.0;
    config_.degree = 3;
    config_.apply_bounds = true;

    spline_ = std::make_unique<RecedingHorizonSpline>(config_);
  }

  BSplinePolicyConfig config_;
  std::unique_ptr<RecedingHorizonSpline> spline_;
};

TEST_F(RecedingHorizonSplineTest, VerboseRun) {
  int n_cpoints = spline_->c_points_.values_.size();
  std::vector<Eigen::ArrayXd> control_polygons;
  for (int i=0; i<config_.samples; i++){
    control_polygons.push_back(spline_->get_sample_control_polygon(i));
  }

  // shifting less then discretization dt should not change the control polygon
  double delta_time = 0.000001;
  spline_->shift(delta_time);
  for (int i=0; i<config_.samples; i++){
    ASSERT_TRUE(control_polygons[i].isApprox(spline_->get_sample_control_polygon(i)));
  }

  // shifting more should shift c-polygon of one point only
  spline_->shift(config_.cp_dt + delta_time);
  for (int i=0; i<config_.samples; i++){
    ASSERT_TRUE(control_polygons[i].tail(n_cpoints-1).isApprox(spline_->get_sample_control_polygon(i).head(n_cpoints-1)));
  }
}

//TEST_F(PathIntegralTest, AsyncRun) {
//  Eigen::Vector2d x0{0.0, 0.0};
//  Eigen::VectorXd u = Eigen::VectorXd::Zero(1);
//
//  config.horizon = 2.0;
//  solver_ = std::make_unique<PathIntegral>(dynamics, cost, sampler, config);
//
//  double t0 = 0.0;
//  std::atomic_bool run_once;
//  run_once = false;
//
//  solver_->set_observation(x0, t0);
//
//  // optimization thread
//  std::thread t1([&, this] {
//    size_t counter = 0;
//    while (counter < 5) {
//      std::cout << "Update #" << counter << std::endl;
//      solver_->update_policy();
//      counter++;
//      std::this_thread::sleep_for(std::chrono::milliseconds(50));
//      std::cout << "Optimal rollout: " << std::endl
//                << solver_->get_optimal_rollout_cache() << std::endl;
//      run_once = true;
//    }
//  });
//
//  // set observation and get control thread
//  std::thread t2([&, this] {
//    size_t counter = 0;
//    double t;
//    while (counter < 5) {
//      if (run_once) {
//        std::cout << "Reset #" << counter << std::endl;
//        t = t0 + counter * config.step_size;
//        solver_->set_observation(x0, t);
//        solver_->get_input(x0, u, t);
//        counter++;
//        std::this_thread::sleep_for(std::chrono::milliseconds(10));
//        std::cout << "u*[" << t << "]: " << u.transpose() << std::endl;
//      }
//    }
//  });
//
//  t1.join();
//  t2.join();
//}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
