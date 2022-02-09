#include "mppi_omav_interaction/state_observer_valve.h"

namespace object_observer {

StateObserverValve::StateObserverValve(const ros::NodeHandle& nh) : nh_(nh) {
  std::string object_pose_topic;
  nh_.param<std::string>("object_pose_topic", object_pose_topic,
                         "/object_pose");
  object_pose_subscriber_ = nh_.subscribe(
      object_pose_topic, 1, &StateObserverValve::objectPoseCallback, this);
  jointStateCallback_ = nh_.subscribe(
      "joint_state_in", 1, &StateObserverValve::jointStateCallback, this);

  // ros publishing
  object_state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("/observer/object/joint_state", 1);

  object_state_.name.push_back("articulation_joint");
  object_state_.position.push_back(0.0);
  object_state_.velocity.push_back(0.0);
  articulation_first_computation_ = true;
}

double unwrap(const double& previous_angle, const double& new_angle) {
  double d = std::fmod(new_angle - previous_angle, 2.0 * M_PI);
  d = d > M_PI ? d - 2.0 * M_PI : (d < -M_PI ? d + 2.0 * M_PI : d);
  return previous_angle + d;
}

bool StateObserverValve::initialize() { return true; }

void StateObserverValve::jointStateCallback(
    const sensor_msgs::JointStateConstPtr& msg) {
  ROS_INFO_THROTTLE(
      1.0,
      "[state_observer_node] Using joint state as input for state observer. "
      "This should only happen in debug mode.");
  double wrapped = std::fmod(msg->position[0], 2.0 * M_PI);
  // Add some random initial rotation and only rotate around valve z axis (for
  // debugging)
  T_world_valve_ = Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1.0, 0.0, 0.0)) *
                   Eigen::AngleAxisd(wrapped, Eigen::Vector3d(0.0, 0.0, 1.0));
  computeAngle(msg->header.stamp);
}

void StateObserverValve::objectPoseCallback(
    const nav_msgs::OdometryConstPtr& msg) {
  tf::poseMsgToEigen(msg->pose.pose, T_world_valve_);
  computeAngle(msg->header.stamp);
}

void StateObserverValve::computeAngle(const ros::Time& stamp) {
  if (articulation_first_computation_) {
    ROS_INFO("First computation of the valve angle.");
    articulation_first_computation_ = false;
    previous_time_ = stamp.toSec();

    T_world_valve_init_ = T_world_valve_;

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped T_world_valve_init;
    tf::transformEigenToMsg(T_world_valve_init_, T_world_valve_init.transform);
    T_world_valve_init.header.stamp = stamp;
    T_world_valve_init.header.frame_id = "world";
    T_world_valve_init.child_frame_id = "valve";
    static_broadcaster.sendTransform(T_world_valve_init);
    ROS_INFO_STREAM("Published initial transform from world to valve frame.");
    std::cout << "Transform is:" << std::endl;
    std::cout << "Translation: " << T_world_valve_.translation().transpose()
              << std::endl;
    std::cout << "Rotation: " << std::endl
              << T_world_valve_.rotation() << std::endl;
    std::cout << "Euler: "
              << T_world_valve_.rotation().eulerAngles(0, 1, 2).transpose()
              << std::endl;
    std::cout << "Euler: "
              << T_world_valve_.rotation().eulerAngles(2, 1, 0).transpose()
              << std::endl;
    return;
  }

  Eigen::Quaterniond rot_init =
      Eigen::Quaterniond(T_world_valve_init_.rotation());
  Eigen::AngleAxisd a_IV(rot_init.inverse() *
                         Eigen::Quaterniond(T_world_valve_.rotation()));
  double theta_new = a_IV.angle();
  // Assume that the valve odom axis has its z axis perpendicular
  if (a_IV.axis()(2) < 0) {
    theta_new = 2.0 * M_PI - theta_new;
  }
  // Unwrap theta:
  theta_new = unwrap(object_state_.position[0], theta_new);

  double current_time = stamp.toSec();

  object_state_.velocity[0] =
      (theta_new - object_state_.position[0]) / (current_time - previous_time_);
  object_state_.position[0] = theta_new;
  previous_time_ = current_time;
}

void StateObserverValve::publish() {
  object_state_.header.stamp = ros::Time(previous_time_);
  object_state_publisher_.publish(object_state_);
}

}  // namespace object_observer