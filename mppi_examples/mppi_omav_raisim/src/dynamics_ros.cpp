/*!
 * @file    dynamics_ros.cpp
 * @author  Matthias Studiger
 * @date    22.03.2021
 * @version 1.0
 * @brief   description
 */

#include "mppi_omav_raisim/dynamics_ros.h"
// TODO: Include the correct message type!
#include <geometry_msgs/PoseStamped.h>

namespace omav_raisim {
    OMAVRaisimDynamicsRos::OMAVRaisimDynamicsRos(const ros::NodeHandle &nh, const std::string &robot_description,
                                                 const double dt)
                                                 : OMAVRaisimDynamics(robot_description, dt) {
        state_publisher_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);

        joint_state_.name = {"omav_pos_x", "omav_pos_y", "omav_pos_z","omav_rot_x", "omav_rot_y",  "omav_rot_z" };
        joint_state_.position.resize(joint_state_.name.size());
        joint_state_.header.frame_id = "world";
    }
    void OMAVRaisimDynamicsRos::reset_to_default() {
        // Set thrust in z-Direction to compensate gravity
        x_.setZero();
        x_(2) = 5*9.81;
        x_(9) = 1;
        reset(x_);
        ROS_INFO_STREAM("Reset simulation ot default value: " << x_.transpose());
    }
    void OMAVRaisimDynamicsRos::publish_ros() {
        // Update robot state visualization
        joint_state_.header.stamp = ros::Time::now();
        for (size_t j = 0; j < 3; j++) {
            joint_state_.position[j] = x_(j+16);
        }
        Eigen::Quaterniond quat = {x_(9), x_(10), x_(11), x_(12)};
        // To publish the attitude of the omav
        auto euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);
        joint_state_.position[3] = euler(0);
        joint_state_.position[4] = euler(1);
        joint_state_.position[5] = euler(2);

        state_publisher_.publish(joint_state_);
    }
} // namespace omav_raisim
