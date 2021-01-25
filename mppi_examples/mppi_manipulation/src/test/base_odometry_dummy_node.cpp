//
// Created by giuseppe on 23.01.21.
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Geometry>

int main(int argc, char** argv){
  ros::init(argc, argv, "base_twist_publisher");
  ros::NodeHandle nh("~");

  std::string twist_topic;
  nh.param<std::string>("twist_topic", twist_topic, "/base_twist");

  ros::Publisher twist_talker = nh.advertise<geometry_msgs::Twist>(twist_topic, 1);
  geometry_msgs::Twist twist_msg;
  twist_msg.linear.x = 0.01;
  twist_msg.linear.y = 0.001;
  twist_msg.angular.z = 0.1;

  tf2_ros::TransformBroadcaster tf_broadcaster;
  geometry_msgs::TransformStamped base_pose;
  base_pose.header.frame_id = "world";
  base_pose.child_frame_id = "base";
  base_pose.transform.translation.x = 0.0;
  base_pose.transform.translation.y = 0.0;
  base_pose.transform.translation.z = 0.0;
  double yaw = 0;

  ros::Rate rate(100);
  while (ros::ok()){
    twist_talker.publish(twist_msg);

    base_pose.transform.translation.x += twist_msg.linear.x * 0.01;
    base_pose.transform.translation.y += twist_msg.linear.y * 0.01;
    yaw += twist_msg.angular.z * 0.01;
    Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    base_pose.transform.rotation.x = q.x();
    base_pose.transform.rotation.y = q.y();
    base_pose.transform.rotation.z = q.z();
    base_pose.transform.rotation.w = q.w();
    base_pose.header.stamp = ros::Time::now();
    tf_broadcaster.sendTransform(base_pose);

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}