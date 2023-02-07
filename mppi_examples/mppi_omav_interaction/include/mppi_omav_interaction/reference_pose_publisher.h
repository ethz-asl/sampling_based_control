#ifndef MPPI_OMAV_INTERACTION_REFERENCE_POSE_PUBLISHER_H
#define MPPI_OMAV_INTERACTION_REFERENCE_POSE_PUBLISHER_H

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include <mppi_omav_interaction/MPPIOmavReferenceConfig.h>
#include <mppi_omav_interaction/ros_conversions.h>

namespace omav_interaction {
/**
 * @brief Reference Pose Publisher. Publishes reference poses for the mppi
 * controller to track.
 */
class ReferencePosePublisher {
 public:
  /**
   * @brief Constructor for ReferencePosePublisher.
   * @param nh ROS node handle
   * @param privat_nh private ROS nodehandle
   */
  ReferencePosePublisher(const ros::NodeHandle& nh,
                         const ros::NodeHandle& private_nh);

 private:
  void referencePoseCallback(
      mppi_omav_interaction::MPPIOmavReferenceConfig& config,
      uint32_t level) const;
  void odometryCallback(const nav_msgs::Odometry& odometry_msg);
  double rad2deg(double angle) const;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher reference_pose_publisher_;
  ros::Subscriber odometry_subscriber_;
  dynamic_reconfigure::Server<mppi_omav_interaction::MPPIOmavReferenceConfig>
      reference_pose_reconfigure_server_;

  mav_msgs::EigenOdometry current_odometry_;
};

inline double ReferencePosePublisher::rad2deg(double angle) const {
  return angle * 360.0 / (2.0 * M_PI);
}
}  // namespace omav_interaction
#endif  // MPPI_OMAV_INTERACTION_REFERENCE_POSE_PUBLISHER_H