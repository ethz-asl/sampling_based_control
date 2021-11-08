#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <chrono>
#include <Eigen/Core>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class Object
{

public:

    Object(const ros::NodeHandle& nh):nh_(nh){};
    void init_publisher(std::string topic_name, int rate);
    void setTF(Eigen::VectorXd state);
    void pub_state();

private:
    ros::NodeHandle nh_;

    ros::Publisher state_publisher_;
    sensor_msgs::JointState state_; 

    ros::Publisher kp_publisher_;
    visualization_msgs::Marker kp_marker_;


public:

    geometry_msgs::TransformStamped trans;
};