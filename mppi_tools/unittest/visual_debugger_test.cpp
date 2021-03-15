#include "mppi_tools/visual_debugger.hpp"
//#include <gtest/gtest.h>
//
// TEST(VisualDebuggerTest, Main) {
//  mppi_tools::VisualDebugger vd{};
//  ASSERT_TRUE(vd.init());
//  vd.render();
//}

int main(int argc, char** argv) {
  mppi_tools::VisualDebugger vd{};
  vd.init();

  size_t n = 20;
  double counter = 0;

  while (true) {
    Eigen::ArrayXd weights(n);
    for (size_t i = 0; i < n; i++) {
      weights(i) = counter + i;
    }
    counter += 0.0001;
    vd.reset_weights(weights);
    if (!vd.render()) return 0;
  }

  //  ::testing::InitGoogleTest(&argc, argv);
  //  return RUN_ALL_TESTS();
  return 0;
}
