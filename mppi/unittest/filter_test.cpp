/*!
 * @file     filter_test.cpp
 * @author   Giuseppe Rizzi
 * @date     09.10.2020
 * @version  1.0
 * @brief    description
 */

#include <gtest/gtest.h>
#include <mppi/filters/savgol_filter.h>
#include <mppi/utils/matplotlibcpp.h>
#include <chrono>
#include <iostream>
#include <random>

TEST(FilterTest, Debug) {
  const int steps = 5;
  const int window_size = 2;
  mppi::MovingExtendedWindow myWindow(steps, window_size);
  std::cout << myWindow << std::endl;

  std::cout << "Adding a point at 0" << std::endl;
  myWindow.trim(0);
  myWindow.add_point(1, 0);
  std::cout << myWindow;

  std::cout << "Trimming at 0.4" << std::endl;
  myWindow.trim(0.4);
  myWindow.add_point(3, 1);
  myWindow.add_point(4, 2);
  myWindow.add_point(5, 3);
  myWindow.add_point(6, 4);
  myWindow.add_point(7, 5);

  std::cout << myWindow << std::endl;

  std::cout << "Extracting at " << 0.5 << std::endl;
  std::vector<double> w1 = myWindow.extract(0.5);
  for (const auto& u : w1) {
    std::cout << u << " ";
  }

  std::cout << "\nExtracting at " << 5 << std::endl;
  std::vector<double> w2 = myWindow.extract(5);
  for (const auto& u : w2) {
    std::cout << u << " ";
  }

  std::cout << "\nTrimming at 2.2" << std::endl;
  myWindow.trim(2.2);
  std::cout << myWindow;

  std::cout << "Adding more points" << std::endl;
  myWindow.add_point(8, 2.2);
  myWindow.add_point(9, 4);
  myWindow.add_point(10, 5);
  std::cout << myWindow;

  std::cout << "Extracting at 3.5" << std::endl;
  std::vector<double> w3 = myWindow.extract(3.5);
  for (const auto& u : w3) {
    std::cout << u << " ";
  }
  std::cout << std::endl;
}

TEST(FilterTest, TestVisual) {
  const int nu = 1;
  const int steps = 100;
  const int window_size = 10;
  const int poly_order = 3;

  std::vector<double> t1(steps, 0.0), t2(steps, 0.0);

  mppi::SavGolFilter filter(steps, nu, window_size, poly_order);
  std::default_random_engine generator;
  std::normal_distribution<double> dist(0.0, 0.1);

  auto start = std::chrono::steady_clock::now();

  Eigen::VectorXd z(1);
  std::vector<double> z_raw1(steps, 0.0);
  constexpr double dt = 0.1;
  double intermediate_t = dt * steps / 2.0;
  double t = 0.0;
  for (size_t i = 0; i < steps; i++) {
    t1[i] = t;
    z(0) = std::sin(t) + dist(generator);
    z_raw1[i] = z(0);
    filter.add_measurement(z, t);
    t += dt;
  }

  Eigen::VectorXd z_filt(1);
  std::vector<double> z_filt_vec(steps, 0.0);
  t = 0.0;
  for (size_t i = 0; i < steps; i++) {
    filter.apply(z_filt, t);
    t += dt;
    z_filt_vec[i] = z_filt(0);
  }

  auto end = std::chrono::steady_clock::now();
  double elapsed =
      std::chrono::duration_cast<std::chrono::microseconds>(end - start)
          .count() /
      1000.0;
  std::cout << "Filtering took: " << elapsed << " ms." << std::endl;

  std::vector<double> z_raw2(steps, 0.0);
  filter.reset(intermediate_t);
  t = intermediate_t;
  for (size_t i = 0; i < steps; i++) {
    t2[i] = t;
    z(0) = std::sin(t) + dist(generator);
    filter.add_measurement(z, t);
    t += dt;
    z_raw2[i] = z(0);
  }

  std::vector<double> z_filt_vec2(steps, 0.0);
  t = intermediate_t;
  for (size_t i = 0; i < steps; i++) {
    filter.apply(z_filt, t);
    t += dt;
    z_filt_vec2[i] = z_filt(0);
  }

  matplotlibcpp::subplot(1, 2, 1);
  matplotlibcpp::named_plot("raw1", t1, z_raw1, "--");
  matplotlibcpp::named_plot("filt 1", t1, z_filt_vec, "o-");
  matplotlibcpp::legend();
  matplotlibcpp::grid(true);

  matplotlibcpp::subplot(1, 2, 2);
  matplotlibcpp::named_plot("raw2", t2, z_raw2, "--");
  matplotlibcpp::named_plot("filt_2", t2, z_filt_vec2, "o-");
  matplotlibcpp::legend();
  matplotlibcpp::grid(true);

  matplotlibcpp::show(true);
}

TEST(FilterTest, TestRealTime) {
  const int nu = 1;
  const int steps = 100;
  const int window_size = 20;
  const int poly_order = 3;

  std::vector<double> t1(steps, 0.0);

  mppi::SavGolFilter filter(steps, nu, window_size, poly_order);
  std::default_random_engine generator;
  std::normal_distribution<double> dist(0.0, 0.1);

  Eigen::VectorXd z(1);
  Eigen::VectorXd z_noisy(1);
  constexpr double dt = 0.1;

  std::vector<double> t_hist;
  std::vector<double> z_hist;
  std::vector<double> z_noisy_hist;
  std::vector<double> z_filt_hist;

  double t = 0.0;
  for (size_t i = 0; i < steps; i++) {
    filter.reset(t);

    z(0) = std::sin(t);
    z_noisy(0) = z(0) + dist(generator);

    t_hist.push_back(t);
    z_hist.push_back(z(0));
    z_noisy_hist.push_back(z_noisy(0));

    filter.add_measurement(z_noisy, t);
    filter.apply(z_noisy, t);
    z_filt_hist.push_back(z_noisy(0));

    t += dt;
  }

  matplotlibcpp::subplot(1, 2, 1);
  matplotlibcpp::named_plot("raw", t_hist, z_noisy_hist, "--");
  matplotlibcpp::named_plot("filt", t_hist, z_filt_hist, "o-");
  matplotlibcpp::legend();
  matplotlibcpp::grid(true);
  matplotlibcpp::show(true);
}

TEST(FilterTest, MultiChannelTest) {
  Eigen::VectorXd u = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd u_filtered = Eigen::VectorXd::Zero(3);
  mppi::SavGolFilter filter(10, 3, {5, 5, 5}, {3, 3, 3});
  double t = 0;
  for (size_t steps = 0; steps < 8; steps++) {
    std::cout << "t: " << t << std::endl;
    if (t == 0.06) u << 1, 1, 1;
    if (t == 0.075) u << 2, 2, 2;
    if (t == 0.9) u << 0, 0, 0;

    std::cout << "In: " << u.transpose() << std::endl;
    filter.reset(t);
    filter.add_measurement(u, t);
    std::cout << "Printing windows" << std::endl;
    for (size_t i = 0; i < u.size(); i++) {
      std::cout << "Window " << i << " " << std::endl;
      std::cout << filter.get_windows()[i];
      std::cout << std::endl;
    }
    filter.apply(u_filtered, t);
    t += 0.015;
  }

  std::cout << "\n\n\nAdding more points without reset\n\n\n";
  u << 4, 4, 4;
  for (size_t steps = 0; steps < 5; steps++) {
    std::cout << "t: " << t << std::endl;
    std::cout << "In: " << u.transpose() << std::endl;
    filter.add_measurement(u, t);
    std::cout << "Printing windows" << std::endl;
    for (size_t i = 0; i < u.size(); i++) {
      std::cout << "Window " << i << " " << std::endl;
      std::cout << filter.get_windows()[i];
      std::cout << std::endl;
    }
    filter.apply(u_filtered, t);
    t += 0.015;
  }

  std::cout
      << "\n\n\n\Trimming at intermediate time and restarting at t=0.13\n\n\n";
  t = 0.13;
  filter.reset(t);
  u << 0, 0, 0;
  for (size_t steps = 0; steps < 8; steps++) {
    std::cout << "t: " << t << std::endl;
    std::cout << "In: " << u.transpose() << std::endl;
    filter.add_measurement(u, t);
    std::cout << "Printing windows" << std::endl;
    for (size_t i = 0; i < u.size(); i++) {
      std::cout << "Window " << i << " " << std::endl;
      std::cout << filter.get_windows()[i];
      std::cout << std::endl;
    }
    filter.apply(u_filtered, t);
    std::cout << "Out: " << u.transpose() << std::endl;
    t += 0.015;
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}