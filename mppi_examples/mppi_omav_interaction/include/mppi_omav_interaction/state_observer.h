//
// Created by studigem on 13.06.21.
//

#ifndef OBJECT_OBSERVER_SHELF_H
#define OBJECT_OBSERVER_SHELF_H

#include <ros/ros.h>
#include <ros/package.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <tf2_ros/static_transform_broadcaster.h>

#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>


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
        bool estimateCenter();

      private:
        void object_pose_callback(const nav_msgs::OdometryConstPtr& msg);


    private:
        ros::NodeHandle nh_;

        // vicon subscribers
        ros::Subscriber object_pose_subscriber_;

        ///// Articulation
        // articulation
        double previous_time_ = 0.0;
        double start_relative_angle_ = 0.0;
        double current_relative_angle_ = 0.0;
        bool articulation_first_computation_ = false;
        sensor_msgs::JointState object_state_;
        ros::Publisher object_state_publisher_;
        tf::TransformListener listener;

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
#endif
