//
// Created by studigem on 13.06.21.
//

#pragma once

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace object_observer {

/// The scope of this class is to collect info and return the complete mobile
/// robot state

    class StateObserver {
    public:
        StateObserver() = delete;
        explicit StateObserver(const ros::NodeHandle& nh);
        ~StateObserver() = default;

    public:
        bool initialize();
        void publish();

    private:
        void object_pose_callback(const nav_msgs::OdometryConstPtr& msg);


    private:
        ros::NodeHandle nh_;

        // vicon subscribers
        ros::Subscriber object_pose_subscriber_;

        ///// Articulation
        // articulation
        double previous_time_;
        double start_relative_angle_;
        double current_relative_angle_;
        bool articulation_first_computation_;
        sensor_msgs::JointState object_state_;
        ros::Publisher object_state_publisher_;

        // Calibration tf
        Eigen::Affine3d T_handle0_shelf_;
        Eigen::Affine3d T_handle0_hinge_;

        // Static tf: the result of calibration (done on first tf)
        Eigen::Affine3d T_world_shelf_;
        Eigen::Affine3d T_hinge_world_;

        // tf required for joint position estimation
        Eigen::Affine3d T_world_handle_;
        Eigen::Affine3d T_hinge_handle_;
        Eigen::Affine3d T_hinge_handle_init_;

    };
}  // namespace object_observer
