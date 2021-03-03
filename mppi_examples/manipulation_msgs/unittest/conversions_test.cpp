//
// Created by giuseppe on 30.01.21.
//

#include <gtest/gtest.h>
#include <manipulation_msgs/State.h>
#include <manipulation_msgs/conversions.h>
#include <Eigen/Core>

TEST(ConversionsTest, eigenToMsg) {
  manipulation_msgs::State stateRos;
  Eigen::Vector3d base_pose;
  Eigen::Vector3d base_twist;
  Eigen::VectorXd arm_position = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd arm_velocity = Eigen::VectorXd::Zero(9);
  double object_position = 0;
  double object_velocity = 0;
  bool contact_state = false;
  manipulation::conversions::toMsg(base_pose, base_twist, arm_position,
                                   arm_velocity, object_position,
                                   object_velocity, contact_state, stateRos);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}