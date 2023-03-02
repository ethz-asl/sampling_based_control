#include "geometry_msgs/TransformStamped.h"
#include "message_filters/subscriber.h"
#include "ros/ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"


class ObjectPoseMockPublisher {
public:
  ObjectPoseMockPublisher(ros::NodeHandle nh) :
    target_frame_("odom_omav"), tf2_listener(tf2_buffer_), nh_(nh),
    tf2_filter_(object_pose_subscriber_, tf2_buffer_, target_frame_, 10, 0)
  {
    object_pose_subscriber_.subscribe(nh_, "/shelf/ground_truth/transform", 10);
    object_pose_publisher_ = nh_.advertise<geometry_msgs::PointStamped>("/shelf/ground_truth_transformed", 1);
    tf2_filter_.registerCallback(boost::bind(&ObjectPoseMockPublisher::callback, this, _1) );
  }

  //  Callback to register with tf2_ros::MessageFilter to be called when transforms are available
  void callback(const geometry_msgs::TransformStampedConstPtr transform_ptr_in) 
  {
    geometry_msgs::TransformStamped transform_out;
    try 
    {
      tf2_buffer_.transform(*transform_ptr_in, transform_out, target_frame_);
      geometry_msgs::PointStamped point_out;
      point_out.header = transform_out.header;
      point_out.point.x = transform_out.transform.translation.x;
      point_out.point.y = transform_out.transform.translation.y;
      point_out.point.z = transform_out.transform.translation.z;
      object_pose_publisher_.publish(point_out);
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
    }
  }

private:
  std::string target_frame_;
  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener;
  ros::NodeHandle nh_;
  message_filters::Subscriber<geometry_msgs::TransformStamped> object_pose_subscriber_;
  tf2_ros::MessageFilter<geometry_msgs::TransformStamped> tf2_filter_;
  ros::Publisher object_pose_publisher_;
};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "object_pose_mock_publisher");
  ros::NodeHandle nh;
  ObjectPoseMockPublisher object_pose_mock_publisher(nh);
  ros::spin();
  return 0;
}