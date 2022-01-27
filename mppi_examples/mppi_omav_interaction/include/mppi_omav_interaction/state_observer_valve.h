#ifndef OBJECT_OBSERVER_VALVE_H
#define OBJECT_OBSERVER_VALVE_H

#include <ros/ros.h>
#include <ros/package.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>

#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tf2_ros/static_transform_broadcaster.h>

namespace object_observer {

/// The scope of this class is to collect info and return the complete mobile
/// robot state

    class StateObserverValve {
    public:
        StateObserverValve() = delete;
        explicit StateObserverValve(const ros::NodeHandle& nh);
        ~StateObserverValve() = default;

    public:
        bool initialize();
        void publish();

      private:
        void objectPoseCallback(const nav_msgs::OdometryConstPtr& msg);
        void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg) ;
        void computeAngle(const ros::Time &stamp);


    private:
        ros::NodeHandle nh_;

        // vicon subscribers
        ros::Subscriber object_pose_subscriber_, jointStateCallback_;

        double previous_time_ = 0.0;
        bool articulation_first_computation_ = false;
        sensor_msgs::JointState object_state_;
        ros::Publisher object_state_publisher_;
        Eigen::Affine3d T_world_valve_,T_world_valve_init_;

    };
}  // namespace object_observer
#endif