//
// Created by giuseppe on 18.08.21.
//

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/JointState.h>

using namespace sensor_msgs;
namespace mf = message_filters;

class JointStateAggregator{
 public:
  JointStateAggregator(ros::NodeHandle& nh, const std::string& arm_topic,
                       const std::string& finger_topic,
                       const std::string& out_topic) {
    state_publisher_ = nh.advertise<JointState>(out_topic, 1);
    arm_state_subscriber_ =
        nh.subscribe(arm_topic, 1, &JointStateAggregator::arm_cb, this);
    finger_state_subscriber_ =
        nh.subscribe(finger_topic, 1, &JointStateAggregator::finger_cb, this);
  }
  ~JointStateAggregator() = default;

 public:
  void aggregate() {
    if (arm_state_.header.stamp.toSec() == previous_time_) return;

    size_t arm_dim = arm_state_.name.size();
    size_t finger_dim = finger_state_.name.size();

    size_t dim = arm_dim + finger_dim;
    sensor_msgs::JointState aggregate_state;
    aggregate_state.name.resize(dim);
    aggregate_state.position.resize(dim);
    aggregate_state.velocity.resize(dim);
    aggregate_state.effort.resize(dim);

    for (int i=0; i<arm_dim; i++){
      aggregate_state.name[i] = arm_state_.name[i];
      aggregate_state.position[i] = arm_state_.position[i];
      aggregate_state.velocity[i] = arm_state_.velocity[i];
      aggregate_state.effort[i] = arm_state_.effort[i];
    }

    for (int i=0; i<finger_dim; i++){
      aggregate_state.name[i + arm_dim] = finger_state_.name[i];
      aggregate_state.position[i + arm_dim] = finger_state_.position[i];
      aggregate_state.velocity[i + arm_dim] = finger_state_.velocity[i];
      aggregate_state.effort[i + arm_dim] = finger_state_.effort[i];
    }

    aggregate_state.header.frame_id = arm_state_.header.frame_id;
    aggregate_state.header.stamp = arm_state_.header.stamp;
    state_publisher_.publish(aggregate_state);
    previous_time_ = arm_state_.header.stamp.toSec();
  }

  void arm_cb(const sensor_msgs::JointStateConstPtr& msg) {
    arm_state_ = *msg;
    arm_state_received_ = true;
  }

  void finger_cb(const sensor_msgs::JointStateConstPtr& msg) {
    finger_state_ = *msg;
    finger_state_received_ = true;
  }

  void run() {
    ros::Rate rate(100);
    while (ros::ok()) {
      if (finger_state_received_ && arm_state_received_) {
        aggregate();
      }
      ros::spinOnce();
      rate.sleep();
    }
  }

 private:
  bool arm_state_received_ = false;
  bool finger_state_received_ = false;

  double previous_time_;
  ros::Publisher state_publisher_;

  sensor_msgs::JointState arm_state_;
  sensor_msgs::JointState finger_state_;

  ros::Subscriber arm_state_subscriber_;
  ros::Subscriber finger_state_subscriber_;
};


int main(int argc, char** argv){
  ros::init(argc, argv, "joint_aggregator");

  ros::NodeHandle nh("~");
  std::string arm_state_topic;
  if (!nh.getParam("arm_state_topic", arm_state_topic)){
    ROS_ERROR("Failed to parse arm_state_topic");
    return 0;
  }

  std::string finger_state_topic;
  if (!nh.getParam("finger_state_topic", finger_state_topic)){
    ROS_ERROR("Failed to parse finger_state_topic");
    return 0;
  }

  std::string output_topic;
  if (!nh.getParam("output_topic", output_topic)){
    ROS_ERROR("Failed to parse output_topic");
    return 0;
  }

  JointStateAggregator aggregator(nh, arm_state_topic, finger_state_topic,
                                  output_topic);
  aggregator.run();
  return 0;
}
