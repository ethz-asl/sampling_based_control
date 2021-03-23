/*!
 * @file    dynamics_ros.h
 * @author  Matthias Studiger
 * @date    19.03.2021
 * @version 1.0
 * @brief   description
 */

#pragma once

#include "mppi_omav_raisim/dynamics.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace omav_raisim {

    class OMAVRaisimDynamicsRos : public OMAVRaisimDynamics {
    public:
        OMAVRaisimDynamicsRos(const ros::NodeHandle& nh,
                              const std::string& robot_description,
                              const double dt);
        ~OMAVRaisimDynamicsRos() = default;

    public:
        void reset_to_default();
        void publish_ros();

    private:
        ros::NodeHandle nh_;
        ros::Publisher state_publisher_;
        // TODO: Remove, if running (not used when implemented on OMAV)
        sensor_msgs::JointState joint_state_;



    };
} // namespace omav_raisim