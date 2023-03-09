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

class ObjectPoseFilter {
 public:
  ObjectPoseFilter(ros::NodeHandle nh, ros::NodeHandle private_nh)
      : tf2_listener(tf2_buffer_),
        nh_(nh),
        private_nh_(private_nh),
        tf2_filter_(object_pose_subscriber_, tf2_buffer_, "", 10, 0),
        filtered_position_(Eigen::Vector3d::Zero()),
        filtered_yaw_(0.0) {
    std::string object_pose_topic;
    private_nh_.getParam("target_frame", target_frame_);
    private_nh_.getParam("object_pose_topic", object_pose_topic);
    private_nh_.getParam("filter_factor", filter_factor_);

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

      Eigen::Vector3d current_position = mav_msgs::vector3FromPointMsg(pose_msg_transformed.pose.position);

      float factor = filter_factor_;
      ros::Duration dt = pose_msg_transformed.header.stamp - previous_timestamp_;
      previous_timestamp_ = pose_msg_transformed.header.stamp;

      if (dt.toSec() > 0.5) {
        factor = 1.0f;
      }

      filtered_position_ = filter_factor_*current_position + (1.0f - filter_factor_) * filtered_position_;

      Eigen::Quaterniond current_orientation = mav_msgs::quaternionFromMsg(pose_msg_transformed.pose.orientation);
      double current_yaw = mav_msgs::yawFromQuaternion(current_orientation);
      filtered_yaw_ = filter_factor_*current_yaw + (1.0f - filter_factor_) * filtered_yaw_;

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
  float filter_factor_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "object_pose_filter");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ObjectPoseFilter object_pose_filter(nh, private_nh);
  ros::spin();
  return 0;
}