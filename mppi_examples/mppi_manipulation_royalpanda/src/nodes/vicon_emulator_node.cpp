//
// Created by giuseppe on 09.02.21.
//
/// Get the reference frame from gazebo and republishes it as odometry msg

#include <gazebo_msgs/LinkStates.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "vicon_emulator_node");
  ros::NodeHandle nh("~");

  nav_msgs::Odometry base_odometry;
  ros::Publisher odometry_publisher = nh.advertise<nav_msgs::Odometry>(
      "/panda_base/vrpn_client/estimated_odometry", 1);

  base_odometry.header.frame_id = "odom";
  base_odometry.child_frame_id = "reference_link";
  std::string link_name = "reference_link_gazebo";

  auto cb = [&](const gazebo_msgs::LinkStatesConstPtr& msg) {
    // Fill odometry message
    auto iter = std::find_if(msg->name.begin(), msg->name.end(),
                             [&](const std::string& str) {
                               return str.find(link_name) != std::string::npos;
                             });
    size_t idx = iter - msg->name.begin();
    if (idx == msg->name.size()) {
      ROS_WARN_STREAM_THROTTLE(
          2.0, "Link: " << *iter << " not found in gazebo link states.");
    } else {
      base_odometry.header.stamp = ros::Time::now();
      base_odometry.pose.pose = msg->pose[idx];
      base_odometry.twist.twist = msg->twist[idx];
      odometry_publisher.publish(base_odometry);
    }
  };

  ros::Subscriber link_subscriber =
      nh.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, cb);
  ros::spin();
  return 0;
}
