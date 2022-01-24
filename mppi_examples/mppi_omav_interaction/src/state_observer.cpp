//
// Created by studigem on 13.06.21.
//

#include "mppi_omav_interaction/state_observer.h"


#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <tf2_ros/static_transform_broadcaster.h>

#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>


namespace object_observer {

    StateObserver::StateObserver(const ros::NodeHandle& nh) : nh_(nh) {
        std::string object_pose_topic;
        nh_.param<std::string>("object_pose_topic", object_pose_topic,
                               "/object_pose");
        object_pose_subscriber_ = nh_.subscribe(
                object_pose_topic, 1, &StateObserver::object_pose_callback, this);

        // ros publishing
        object_state_publisher_ =
                nh_.advertise<sensor_msgs::JointState>("/observer/object/joint_state", 1);

        object_state_.name.push_back("articulation_joint");
        object_state_.position.push_back(0.0);
        object_state_.velocity.push_back(0.0);
        articulation_first_computation_ = true;
    }

    bool StateObserver::initialize() {
        KDL::Tree object_kinematics;
        if (!kdl_parser::treeFromParam("object_description", object_kinematics)) {
            ROS_ERROR("Failed to create KDL::Tree from 'object_description'");
            return false;
        }

        KDL::Chain chain;
        object_kinematics.getChain("shelf", "handle_link", chain);
        if (chain.getNrOfJoints() != 1) {
            ROS_ERROR("The object has more then one joint. Only one joint supported!");
            return false;
        }

        // at start-up door is closed
        KDL::JntArray joint_pos(chain.getNrOfJoints());

        // required to calibrate the initial shelf position
        KDL::Frame T_shelf_handle_KDL;
        KDL::ChainFkSolverPos_recursive fk_solver_shelf(chain);
        fk_solver_shelf.JntToCart(joint_pos, T_shelf_handle_KDL);
        tf::transformKDLToEigen(T_shelf_handle_KDL.Inverse(), T_handle0_shelf_);

        // required to now the origin hinge position
        KDL::Frame T_door_handle_KDL;
        object_kinematics.getChain("axis_link", "handle_link", chain);
        KDL::ChainFkSolverPos_recursive fk_solver_hinge(chain);
        fk_solver_hinge.JntToCart(joint_pos, T_door_handle_KDL);
        tf::transformKDLToEigen(T_door_handle_KDL.Inverse(), T_handle0_hinge_);


        ROS_INFO_STREAM("Static transformations summary: "
                                << std::endl
                                << " T_shelf_handle:\n "
                                << T_handle0_shelf_.inverse().matrix() << std::endl
                                << " T_hinge_handle:\n "
                                << T_handle0_hinge_.inverse().matrix() << std::endl);
        ROS_INFO("Robot observer correctly initialized.");
        return true;
    }

    void StateObserver::object_pose_callback(
            const nav_msgs::OdometryConstPtr& msg) {
        tf::poseMsgToEigen(msg->pose.pose, T_world_handle_);
        if (articulation_first_computation_) {
            ROS_INFO("First computation of the shelf pose.");
            T_world_shelf_ = T_world_handle_ * T_handle0_shelf_;
            T_hinge_world_ = (T_world_handle_ * T_handle0_hinge_).inverse();
            T_hinge_handle_init_ = T_hinge_world_ * T_world_handle_;

            start_relative_angle_ = std::atan2(T_hinge_handle_init_.translation().x(),
                                               T_hinge_handle_init_.translation().y());
            articulation_first_computation_ = false;
            previous_time_ = msg->header.stamp.toSec();

            static tf2_ros::StaticTransformBroadcaster static_broadcaster;
            geometry_msgs::TransformStamped T_world_shelf_ros;
            tf::transformEigenToMsg(T_world_shelf_, T_world_shelf_ros.transform);
            T_world_shelf_ros.header.stamp = ros::Time::now();
            T_world_shelf_ros.header.frame_id = "world";
            T_world_shelf_ros.child_frame_id = "shelf";
            static_broadcaster.sendTransform(T_world_shelf_ros);
            ROS_INFO_STREAM("Published initial transform from world to shelf frame.");
            return;
        }

        T_hinge_handle_ = T_hinge_world_ * T_world_handle_;
        current_relative_angle_ = std::atan2(T_hinge_handle_.translation().x(),
                                             T_hinge_handle_.translation().y());

        double theta_new = current_relative_angle_ - start_relative_angle_;
        double current_time = msg->header.stamp.toSec();

        object_state_.velocity[0] =
                (theta_new - object_state_.position[0]) / (current_time - previous_time_);
        object_state_.position[0] = theta_new;
        previous_time_ = current_time;
    }

    bool StateObserver::estimateCenter() {
      tf::StampedTransform transform;

      Eigen::MatrixXf regressor = Eigen::MatrixXf::Zero(20, 3);
      Eigen::VectorXf y = Eigen::VectorXf::Zero(20);
      Eigen::Vector3f params;
      listener.waitForTransform("/world", "/handle_link", ros::Time(0),
                                ros::Duration(3.0));
      int i = 0;
      while (i < 20) {
        listener.lookupTransform("/world", "/handle_link", ros::Time(0),
                                 transform);
        if (transform.getOrigin().y() > -0.9) {
          regressor.row(i) << 2 * transform.getOrigin().x(),
              2 * transform.getOrigin().y(), 1;
          y(i) = transform.getOrigin().x() * transform.getOrigin().x() +
                 transform.getOrigin().y() * transform.getOrigin().y();
          ros::Duration(0.2).sleep();
          i += 1;
        }
      }
      params =
          regressor.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y);
      std::cout << "XM = " << params(0) << std::endl;
      std::cout << "YM = " << params(1) << std::endl;
      std::cout << "Param(2)" << params(2) << std::endl;
      std::cout << "Radius = "
                << std::sqrt(params(2) + pow(params(0), 2) + pow(params(1), 2))
                << std::endl;
      return true;
    }

    void StateObserver::publish() {
        object_state_.header.stamp = ros::Time(previous_time_);
        object_state_publisher_.publish(object_state_);
    }

}  // namespace royalpanda