/*!
 * @file     multivariate_normal_eigen_test.cpp
 * @author   Giuseppe Rizzi
 * @date     21.07.2020
 * @version  1.0
 * @brief    description
 */

#include "mppi/sampler/multivariate_normal_eigen.h"
#include <gtest/gtest.h>
#include <math.h>
#include <Eigen/Dense>
#include <array>
#include <chrono>

TEST(MultivaritateNormal, Sample1D) {
  Eigen::MatrixXd covar(1, 1);
  Eigen::VectorXd mean(1);
  covar << 10.0;
  mean << 3.0;
  auto mn = mppi::multivariate_normal(mean, covar);
  ASSERT_TRUE(mn().size() == 1);

  std::map<int, int> hist{};
  for (size_t i = 0; i < 10000; i++) ++hist[std::round(mn()(0))];

  std::cout << "Mean=" << mean(0) << ", covar=" << covar(0, 0) << std::endl;
  for (auto p : hist) {
    std::cout << std::setw(2) << p.first << ' '
              << std::string(p.second / 10, '*') << '\n';
  }
}

TEST(MultivaritateNormal, Sample2D) {
  Eigen::MatrixXd covar(2, 2);
  Eigen::VectorXd mean(2);
  covar << 20.0, 0.0, 0.0, 5.0;
  mean << 3.0, -1.0;
  auto mn = mppi::multivariate_normal(mean, covar);
  ASSERT_TRUE(mn().size() == 2);

  std::map<int, int> hist1{};
  std::map<int, int> hist2{};

  for (size_t i = 0; i < 10000; i++) {
    ++hist1[std::round(mn()(0))];
    ++hist2[std::round(mn()(1))];
  }
  std::cout << std::endl;

  std::cout << "First variable: mean=" << mean(0) << ", covar=" << covar(0, 0)
            << std::endl;
  for (auto p : hist1) {
    std::cout << std::setw(2) << p.first << ' '
              << std::string(p.second / 10, '*') << '\n';
  }

  std::cout << "Second variable: mean=" << mean(1) << ", covar=" << covar(1, 1)
            << std::endl;
  for (auto p : hist2) {
    std::cout << std::setw(2) << p.first << ' '
              << std::string(p.second / 10, '*') << '\n';
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
