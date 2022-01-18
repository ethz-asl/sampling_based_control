//
// Created by giuseppe on 18.08.21.
//

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace sensor_msgs;
namespace mf = message_filters;

class JointStateAggregator {
 public:
  JointStateAggregator(ros::NodeHandle& nh, const std::string& arm_topic,
                       const std::string& finger_topic,
                       const std::string& out_topic) {
    state_publisher_ = nh.advertise<JointState>(out_topic, 1);
    finger_sub_ = nh.subscribe(finger_topic, 1,
                               &JointStateAggregator::finger_callback, this);
    arm_sub_ =
        nh.subscribe(arm_topic, 1, &JointStateAggregator::arm_callback, this);
  }
  ~JointStateAggregator() = default;

 private:
  ros::Subscriber finger_sub_;
  ros::Subscriber arm_sub_;
  sensor_msgs::JointState arm_state;
  sensor_msgs::JointState finger_state;

 public:
  void arm_callback(const sensor_msgs::JointStateConstPtr& msg) {
    arm_state = *msg;
  }

  void finger_callback(const sensor_msgs::JointStateConstPtr& msg) {
    finger_state = *msg;
  }

  void publish() {
    if (arm_state.header.stamp.toSec() == previous_time_) return;

    size_t arm_dim = arm_state.name.size();
    size_t finger_dim = finger_state.name.size();

    size_t dim = arm_dim + finger_dim;
    sensor_msgs::JointState aggregate_state;
    aggregate_state.name.resize(dim);
    aggregate_state.position.resize(dim);
    aggregate_state.velocity.resize(dim);
    aggregate_state.effort.resize(dim);

    for (int i = 0; i < arm_dim; i++) {
      aggregate_state.name[i] = arm_state.name[i];
      aggregate_state.position[i] = arm_state.position[i];
      aggregate_state.velocity[i] = arm_state.velocity[i];
      aggregate_state.effort[i] = arm_state.effort[i];
    }

    for (int i = 0; i < finger_dim; i++) {
      aggregate_state.name[i + arm_dim] = finger_state.name[i];
      aggregate_state.position[i + arm_dim] = finger_state.position[i];
      aggregate_state.velocity[i + arm_dim] = finger_state.velocity[i];
      aggregate_state.effort[i + arm_dim] = finger_state.effort[i];
    }

    aggregate_state.header.frame_id = arm_state.header.frame_id;
    aggregate_state.header.stamp = arm_state.header.stamp;
    state_publisher_.publish(aggregate_state);
    previous_time_ = arm_state.header.stamp.toSec();
  }

 private:
  double previous_time_;
  ros::Publisher state_publisher_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "joint_aggregator");

  ros::NodeHandle nh("~");
  std::string arm_state_topic;
  if (!nh.getParam("arm_state_topic", arm_state_topic)) {
    ROS_ERROR("Failed to parse arm_state_topic");
    return 0;
  }

  std::string finger_state_topic;
  if (!nh.getParam("finger_state_topic", finger_state_topic)) {
    ROS_ERROR("Failed to parse finger_state_topic");
    return 0;
  }

  std::string output_topic;
  if (!nh.getParam("output_topic", output_topic)) {
    ROS_ERROR("Failed to parse output_topic");
    return 0;
  }

  // bool exact_sync = false;
  // if (!nh.getParam("exact_sync", exact_sync)) {
  //   ROS_ERROR("Failed to parse arm_state_topic");
  //   return 0;
  // }

  JointStateAggregator aggregator(nh, arm_state_topic, finger_state_topic,
                                  output_topic);

  ros::Rate rate(100);
  while (ros::ok()) {
    aggregator.publish();
    rate.sleep();
    ros::spinOnce();
  }

  // mf::Subscriber<JointState> arm_sub(nh, arm_state_topic, 1);
  // mf::Subscriber<JointState> finger_sub(nh, finger_state_topic, 1);

  // typedef mf::sync_policies::ExactTime<JointState, JointState> ExactPolicy;
  // mf::Synchronizer<ExactPolicy> exactSync(ExactPolicy(10), arm_sub,
  // finger_sub);

  // typedef mf::sync_policies::ApproximateTime<JointState, JointState>
  //     ApproximatePolicy;
  // mf::Synchronizer<ApproximatePolicy> approximateSync(ApproximatePolicy(1),
  //                                                     arm_sub, finger_sub);

  // if (exact_sync) {
  //   exactSync.registerCallback(
  //       boost::bind(&JointStateAggregator::callback, &aggregator, _1, _2));
  // } else {
  //   approximateSync.registerCallback(
  //       boost::bind(&JointStateAggregator::callback, &aggregator, _1, _2));
  // }

  // ros::spin();
  return 0;
}
