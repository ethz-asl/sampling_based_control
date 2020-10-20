/*!
 * @file     filter_test.cpp
 * @author   Giuseppe Rizzi
 * @date     09.10.2020
 * @version  1.0
 * @brief    description
 */

#include <mppi/filters/savgol_filter.h>
#include <gtest/gtest.h>
#include <iostream>

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

  std::cout << "\n\nExtracting at " << 5 << std::endl;
  std::vector<double> w2 = myWindow.extract(5);
  for(const auto& u : w2){
    std::cout << u << " ";
  }

  std::cout << "Trimming at 2.2" << std::endl;
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

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}