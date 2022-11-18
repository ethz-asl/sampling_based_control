#include "gtest/gtest.h"

#include <mppi_omav_interaction/ros_conversions.h>

static constexpr double ABS_ERROR = 1e-6;

TEST(RosConversionsTest, RPYtoQuaternion) {
  double roll = 18.072;
  double pitch = -41.539;
  double yaw = -26.703;

  Eigen::Quaterniond quaternion;

  omav_interaction::conversions::RPYtoQuaterniond(roll, pitch, yaw, quaternion);

  ASSERT_NEAR(quaternion.w(), 0.9113136, ABS_ERROR);
  ASSERT_NEAR(quaternion.x(), 0.062008, ABS_ERROR);
  ASSERT_NEAR(quaternion.y(), -0.3746539, ABS_ERROR);
  ASSERT_NEAR(quaternion.z(), -0.1590502, ABS_ERROR);

  roll = -45.0;
  pitch = 80.0;
  yaw = -270.0;

  omav_interaction::conversions::RPYtoQuaterniond(roll, pitch, yaw, quaternion);

  ASSERT_NEAR(quaternion.w(), -0.3265056, ABS_ERROR);
  ASSERT_NEAR(quaternion.x(), 0.6272114, ABS_ERROR);
  ASSERT_NEAR(quaternion.y(), -0.2126311, ABS_ERROR);
  ASSERT_NEAR(quaternion.z(), -0.6743797, ABS_ERROR);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}