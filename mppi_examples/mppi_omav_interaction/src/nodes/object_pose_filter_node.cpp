#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "message_filters/subscriber.h"
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

class ObjectPoseFilter {
 public:
  ObjectPoseFilter(ros::NodeHandle nh, ros::NodeHandle private_nh)
      : tf2_listener(tf2_buffer_),
        nh_(nh),
        private_nh_(private_nh),
        tf2_filter_(object_pose_subscriber_, tf2_buffer_, "", 10, 0) {
    std::string object_pose_topic;
    private_nh_.getParam("target_frame", target_frame_);
    private_nh_.getParam("object_pose_topic", object_pose_topic);

    object_pose_subscriber_.subscribe(nh_, object_pose_topic, 10);
    object_pose_publisher_ =
        private_nh_.advertise<geometry_msgs::PoseStamped>("pose_filtered", 1);

    tf2_filter_.setTargetFrame(target_frame_);
    tf2_filter_.registerCallback(
        boost::bind(&ObjectPoseFilter::callback, this, _1));
  }

  void callback(const geometry_msgs::PoseStampedConstPtr pose_msg_ptr_in) {
    geometry_msgs::PoseStamped pose_msg_out;
    try {
      tf2_buffer_.transform(*pose_msg_ptr_in, pose_msg_out, target_frame_);
      object_pose_publisher_.publish(pose_msg_out);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("[object_pose_filter] Failure %s\n", ex.what());
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
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "object_pose_filter");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ObjectPoseFilter object_pose_filter(nh, private_nh);
  ros::spin();
  return 0;
}