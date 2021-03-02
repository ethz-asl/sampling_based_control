/*!
 * @file     thread_pool.cpp
 * @author   Giuseppe Rizzi
 * @date     28.07.2020
 * @version  1.0
 * @brief    description
 */

#include <gtest/gtest.h>
#include <chrono>
#include <iostream>

#include "mppi/utils/thread_pool.h"

class TestClass {
 public:
  TestClass() { v.resize(1000000, 0); }
  size_t v_size = 1000000;
  std::vector<double> v;

  void add_one(int idx_start = 0, int idx_end = 1000000) {
    std::cout << "Function called " << std::endl;
    for (size_t i = idx_start; i < idx_end; i++) v[i] = v[i] + 1;
    std::cout << "End of function." << std::endl;
  }

  void print() {
    for (int i = 0; i < v.size(); i++) std::cout << v[i] << " ";
    std::cout << std::endl;
  }
};

TEST(ThreadPoolTest, ParallelFun) {
  TestClass tc;
  ThreadPool pool(8);
  int chunk = 250000;
  // tc.print();
  std::vector<std::future<void>> vf;

  auto start0 = std::chrono::steady_clock::now();
  for (size_t i = 0; i < 8; i++)
    vf.push_back(
        pool.enqueue(std::bind(&TestClass::add_one, &tc, std::placeholders::_1,
                               std::placeholders::_2),
                     i * chunk, (i + 1) * chunk));
  for (auto& f : vf) f.get();
  auto end0 = std::chrono::steady_clock::now();
  std::cout << "Synced" << std::endl;
  // tc.print();

  auto start1 = std::chrono::steady_clock::now();
  tc.add_one();
  auto end1 = std::chrono::steady_clock::now();

  std::cout << "Multithreading took "
            << std::chrono::duration_cast<std::chrono::microseconds>(end0 -
                                                                     start0)
                   .count()
            << " us." << std::endl;
  std::cout << "Sing thread took "
            << std::chrono::duration_cast<std::chrono::microseconds>(end1 -
                                                                     start1)
                   .count()
            << " us." << std::endl;

  // tc.print();
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}