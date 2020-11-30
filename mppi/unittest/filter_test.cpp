/*!
 * @file     filter_test.cpp
 * @author   Giuseppe Rizzi
 * @date     09.10.2020
 * @version  1.0
 * @brief    description
 */

#include <mppi/filters/savgol_filter.h>
#include <mppi/utils/matplotlibcpp.h>
#include <gtest/gtest.h>
#include <iostream>
#include <random>
#include <chrono>

TEST(FilterTest, Debug){
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
  for(const auto& u : w1){
    std::cout << u << " ";
  }

  std::cout << "\nExtracting at " << 5 << std::endl;
  std::vector<double> w2 = myWindow.extract(5);
  for(const auto& u : w2){
    std::cout << u << " ";
  }

  std::cout << "\nTrimming at 2.2" << std::endl;
  myWindow.trim(2.2);
  std::cout << myWindow ;

  std::cout << "Adding more points" << std::endl;
  myWindow.add_point(8, 2.2);
  myWindow.add_point(9, 4);
  myWindow.add_point(10, 5);
  std::cout << myWindow;

  std::cout << "Extracting at 3.5" << std::endl;
  std::vector<double> w3 = myWindow.extract(3.5);
  for(const auto& u : w3){
    std::cout << u << " ";
  }
  std::cout << std::endl;
}

TEST(FilterTest, TestVisual){
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
  double intermediate_t = dt * steps/2.0;
  double t = 0.0;
  for(size_t i=0; i<steps; i++){
    t1[i] = t;
    z(0) = std::sin(t) + dist(generator);
    z_raw1[i] = z(0);
    filter.add_measurement(z, t);
    t += dt;
  }

  Eigen::VectorXd z_filt(1);
  std::vector<double> z_filt_vec(steps, 0.0);
  t = 0.0;
  for(size_t i=0; i<steps; i++){
    filter.apply(z_filt, t);
    t += dt;
    z_filt_vec[i] = z_filt(0);
  }

  auto end = std::chrono::steady_clock::now();
  double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count()/1000.0;
  std::cout << "Filtering took: " << elapsed << " ms." << std::endl;

  std::vector<double> z_raw2(steps, 0.0);
  filter.reset(intermediate_t);
  t = intermediate_t;
  for(size_t i=0; i<steps; i++){
    t2[i] = t;
    z(0) = std::sin(t) + dist(generator);
    filter.add_measurement(z, t);
    t += dt;
    z_raw2[i] = z(0);
  }


  std::vector<double> z_filt_vec2(steps, 0.0);
  t = intermediate_t;
  for(size_t i=0; i<steps; i++){
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


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}