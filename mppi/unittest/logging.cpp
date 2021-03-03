/*!
 * @file     logging.cpp
 * @author   Giuseppe Rizzi
 * @date     11.08.2020
 * @version  1.0
 * @brief    description
 */

#include <gtest/gtest.h>
#include <chrono>
#include <iostream>

#include "mppi/utils/logging.h"

TEST(TestLogging, Logging) {
  mppi::log_info("This is and info.");
  mppi::log_warning("This is a warning.");
  mppi::log_error("This is an error.");
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}