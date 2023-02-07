#include <ros/ros.h>

#include <mppi_omav_interaction/reference_pose_publisher.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "reference_pose_publisher_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  omav_interaction::ReferencePosePublisher reference_pose_publisher(nh,
                                                                    private_nh);

  ros::spin();

  return 0;
}