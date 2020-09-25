/*!
 * @file     filtering_test.cpp
 * @author   Giuseppe Rizzi
 * @date     27.07.2020
 * @version  1.0
 * @brief    description
 */

#include <mppi/filters/savgol_filter.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/node_handle.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "filter_test_node");
  ros::NodeHandle nh("~");
  int window_size = nh.param<int>("window_size", 1);
  int filter_order = nh.param<int>("filter_order", 1);
  auto bag_name = nh.param<std::string>("bag_name", "");
  ROS_INFO_STREAM("Filter test " << std::endl
    << "window size:  " << window_size << std::endl
    << "filter order: " << filter_order << std::endl
    << "bag file:     " << bag_name);

  constexpr int channels = 7;
  mppi::SavGolFilter filter(channels, window_size, filter_order);
  ros::Publisher filter_in_publisher = nh.advertise<std_msgs::Float32MultiArray>("/input", 10);
  ros::Publisher filter_out_publisher = nh.advertise<std_msgs::Float32MultiArray>("/input_filtered", 10);

  rosbag::Bag bag;
  try{
    bag.open(bag_name, rosbag::bagmode::Read);
  }
  catch (rosbag::BagException& exc){
    ROS_ERROR_STREAM(exc.what());
    return -1;
  }

  Eigen::VectorXd u_in = Eigen::VectorXd::Zero(channels);
  std_msgs::Float32MultiArray u_out;
  u_out.data.resize(channels);

  std::vector<std::string> topics{"/input"};
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  ros::Duration(3.0).sleep();
  size_t c=0;
  for(rosbag::MessageInstance const m: rosbag::View(bag))
  {
    std_msgs::Float32MultiArray::ConstPtr msg = m.instantiate<std_msgs::Float32MultiArray>();
    if (msg != nullptr){
        for (size_t i=0; i<msg->data.size(); i++)
          u_in(i) = msg->data[i];
        filter.filter(u_in);
        for (size_t i=0; i<msg->data.size(); i++)
          u_out.data[i] = u_in(i);
        filter_in_publisher.publish(*msg);
        filter_out_publisher.publish(u_out);
        c++;
        ros::Duration(0.01).sleep();
    }
  }
  ROS_INFO_STREAM("Total messages: " << c);
  bag.close();
  return 0;
}
