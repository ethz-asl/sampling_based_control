#include "gtest/gtest.h"

#include <geometry_msgs/Pose.h>

#include "mppi_omav_interaction/ros_conversions.h"

static constexpr double ABS_ERROR = 1e-6;

TEST(RosConversionsTest, RPYtoQuaternion) {
  double roll = 18.072;
  double pitch = -41.539;
  double yaw = -26.703;

  Eigen::Quaterniond quaternion;

  omav_interaction::conversions::RPYtoQuaterniond(roll, pitch, yaw, quaternion);

  EXPECT_NEAR(quaternion.w(), 0.9113136, ABS_ERROR);
  EXPECT_NEAR(quaternion.x(), 0.062008, ABS_ERROR);
  EXPECT_NEAR(quaternion.y(), -0.3746539, ABS_ERROR);
  EXPECT_NEAR(quaternion.z(), -0.1590502, ABS_ERROR);

  roll = -45.0;
  pitch = 80.0;
  yaw = -270.0;

  omav_interaction::conversions::RPYtoQuaterniond(roll, pitch, yaw, quaternion);

  EXPECT_NEAR(quaternion.w(), -0.3265056, ABS_ERROR);
  EXPECT_NEAR(quaternion.x(), 0.6272114, ABS_ERROR);
  EXPECT_NEAR(quaternion.y(), -0.2126311, ABS_ERROR);
  EXPECT_NEAR(quaternion.z(), -0.6743797, ABS_ERROR);
}

TEST(RosConversionTest, vectorFromPoseMsg) {
  geometry_msgs::Pose pose_msg;
  pose_msg.position.x = 2.3;
  pose_msg.position.y = -1.23;
  pose_msg.position.z = 0.75;
  pose_msg.orientation.w = 0.268;
  pose_msg.orientation.x = -0.003;
  pose_msg.orientation.y = -0.914;
  pose_msg.orientation.z = 0.305;

  Eigen::Matrix<double, 7, 1> pose_eigen;
  omav_interaction::conversions::vectorFromPoseMsg(pose_msg, pose_eigen);
  EXPECT_FLOAT_EQ(pose_msg.position.x, pose_eigen[0]);
  EXPECT_FLOAT_EQ(pose_msg.position.y, pose_eigen[1]);
  EXPECT_FLOAT_EQ(pose_msg.position.z, pose_eigen[2]);
  EXPECT_FLOAT_EQ(pose_msg.orientation.w, pose_eigen[3]);
  EXPECT_FLOAT_EQ(pose_msg.orientation.x, pose_eigen[4]);
  EXPECT_FLOAT_EQ(pose_msg.orientation.y, pose_eigen[5]);
  EXPECT_FLOAT_EQ(pose_msg.orientation.z, pose_eigen[6]);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}