#include "mppi_tools/visual_debugger.hpp"
#include <gtest/gtest.h>

TEST(VisualDebuggerTest, Main) {
  mppi_tools::VisualDebugger vd{};
  ASSERT_TRUE(vd.init());
  vd.render();
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
