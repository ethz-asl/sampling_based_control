#include <cmath>
#include <deque>
#include <Eigen/Core>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "message_filters/subscriber.h"
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

#include "mav_msgs/common.h"

struct FilterParameters {
  struct Coefficients {
    double position;
    double orientation;
  } coefficients;

  struct Thresholds {
    double position;
    double orientation;
  } thresholds;

  double timeout;
};

class ObjectPoseFilter {
 public:
  ObjectPoseFilter(ros::NodeHandle nh, ros::NodeHandle private_nh)
      : tf2_listener(tf2_buffer_),
        nh_(nh),
        private_nh_(private_nh),
        tf2_filter_(object_pose_subscriber_, tf2_buffer_, "", 10, 0),
        filtered_position_(Eigen::Vector3d::Zero()),
        filtered_yaw_(0.0),
        filter_initialized_(false) {
    std::string object_pose_topic;
    private_nh_.getParam("target_frame", target_frame_);
    private_nh_.getParam("object_pose_topic", object_pose_topic);

    // get parameters
    private_nh_.getParam("position_filter_coeff",
                         filter_parameters_.coefficients.position);
    private_nh_.getParam("orientation_filter_coeff",
                         filter_parameters_.coefficients.orientation);
    private_nh_.getParam("filter_timeout", filter_parameters_.timeout);
    private_nh_.getParam("position_threshold",
                         filter_parameters_.thresholds.position);
    private_nh_.getParam("orientation_threshold",
                         filter_parameters_.thresholds.orientation);

    object_pose_subscriber_.subscribe(nh_, object_pose_topic, 10);
    object_pose_publisher_ =
        private_nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_filtered", 1);

    tf2_filter_.setTargetFrame(target_frame_);
    tf2_filter_.registerCallback(
        boost::bind(&ObjectPoseFilter::callback, this, _1));
  }

  void callback(const geometry_msgs::PoseStampedConstPtr pose_msg_ptr_in) {
    geometry_msgs::PoseStamped pose_msg_transformed;
    try {
      tf2_buffer_.transform(*pose_msg_ptr_in, pose_msg_transformed, target_frame_);

      float position_coeff = filter_parameters_.coefficients.position;
      float orientation_coeff = filter_parameters_.coefficients.orientation;

      // reset filter
      ros::Duration dt = pose_msg_transformed.header.stamp - previous_timestamp_;
      previous_timestamp_ = pose_msg_transformed.header.stamp;

      if (dt.toSec() > filter_parameters_.timeout) {
        position_coeff = 1.0f;
        orientation_coeff = 1.0f;
      }

      // Position
      Eigen::Vector3d current_position =
          mav_msgs::vector3FromPointMsg(pose_msg_transformed.pose.position);

      // Gram-Schmitt
      Eigen::Quaterniond current_orientation =
          mav_msgs::quaternionFromMsg(pose_msg_transformed.pose.orientation);
      Eigen::Matrix3d current_object_frame =
          current_orientation.toRotationMatrix();
      Eigen::Vector3d world_unit_vector_z = {0.0, 0.0, 1.0};
      Eigen::Vector3d object_unit_vector_y = current_object_frame.col(1);
      Eigen::Vector3d object_unit_vector_x =
          object_unit_vector_y.cross(world_unit_vector_z);
      object_unit_vector_x.normalize();
      object_unit_vector_y = -object_unit_vector_x.cross(world_unit_vector_z);
      object_unit_vector_y.normalize();
      Eigen::Matrix3d current_object_frame_aligned;
      current_object_frame_aligned << object_unit_vector_x,
          object_unit_vector_y, world_unit_vector_z;
      Eigen::Quaterniond current_orientation_aligned(
          current_object_frame_aligned);

      double current_yaw =
          mav_msgs::yawFromQuaternion(current_orientation_aligned);

      // initialize filter
      if (!filter_initialized_) {
        ROS_INFO("[%s] Initialize filter", ros::this_node::getName().c_str());
        filtered_position_ = current_position;
        filtered_yaw_ = current_yaw;
        filter_initialized_ = true;
      }

      // filter
      Eigen::Vector3d delta_position = current_position - filtered_position_;
      if (delta_position.norm() < filter_parameters_.thresholds.position) {
        filtered_position_ = position_coeff * current_position +
                             (1.0f - position_coeff) * filtered_position_;
      }

      double delta_yaw = current_yaw - filtered_yaw_;
      if (std::abs(delta_yaw) < filter_parameters_.thresholds.orientation) {
        filtered_yaw_ = orientation_coeff * current_yaw +
                        (1.0f - orientation_coeff) * filtered_yaw_;
      }

      // Publish ros message
      geometry_msgs::PoseWithCovarianceStamped pose_msg_filtered;
      pose_msg_filtered.header = pose_msg_transformed.header;
      pose_msg_filtered.pose.pose.position.x = filtered_position_.x();
      pose_msg_filtered.pose.pose.position.y = filtered_position_.y();
      pose_msg_filtered.pose.pose.position.z = filtered_position_.z();
      pose_msg_filtered.pose.pose.orientation.w = std::cos(filtered_yaw_/2.0);
      pose_msg_filtered.pose.pose.orientation.x = 0.0;
      pose_msg_filtered.pose.pose.orientation.y = 0.0;
      pose_msg_filtered.pose.pose.orientation.z = std::sin(filtered_yaw_/2.0);

      object_pose_publisher_.publish(pose_msg_filtered);

      
    } catch (tf2::TransformException &ex) {
      ROS_WARN("[object_pose_filter] Failure: %s\n", ex.what());
    }
  }

 private:
  std::string target_frame_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  message_filters::Subscriber<geometry_msgs::PoseStamped>
      object_pose_subscriber_;
  tf2_ros::MessageFilter<geometry_msgs::PoseStamped> tf2_filter_;
  ros::Publisher object_pose_publisher_;

  ros::Time previous_timestamp_;
  Eigen::Vector3d filtered_position_;
  double filtered_yaw_;
  bool filter_initialized_;
  FilterParameters filter_parameters_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "object_pose_filter");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ObjectPoseFilter object_pose_filter(nh, private_nh);
  ros::spin();
  return 0;
}