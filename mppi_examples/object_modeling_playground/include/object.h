#pragma once
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <chrono>
#include <Eigen/Core>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <interactive_markers/interactive_marker_server.h>


class Object
{
public:
    Object(const ros::NodeHandle& nh);
    ~Object() = default;
    virtual void update_obj_TF(){};
    virtual void pub_state(){};
    virtual void update_kp_markers(){};
    virtual void fit_primitive(){};
    virtual void vis_primitive(){};
    virtual void pose_estimate(){};

    geometry_msgs::TransformStamped obj_trans;
    geometry_msgs::TransformStamped kp_trans;

protected:
    bool init_param();
    void kp_int_callback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void init_kp_TF(); // init keypoints frame TF

    // obj config vars
    std::string ref_frame;
    std::string object_name;
    std::vector<double> obj_scale_;
    std::vector<double> obj_pos_;
    std::vector<double> obj_rot_;
    int primitive_num = 0 ;

    // kp vars
    int kp_num;
    std::vector<double> keypoints_;
    std::vector<double> keypoints_past_;

    // ros vars
    ros::Subscriber kp_int_subscriber_;

    ros::Publisher state_publisher_;
    sensor_msgs::JointState state_; 

    ros::Publisher kp_publisher_;
    visualization_msgs::Marker kp_marker_;
    visualization_msgs::MarkerArray kp_markers_;

    ros::Publisher primitive_publisher_;
    visualization_msgs::MarkerArray primitive_markers_;
    visualization_msgs::Marker primitive_marker_;



    int bottom_ind;
    std::vector<double> height;
    std::vector<double> radius;

private:
    // params of approx primitive
    ros::NodeHandle nh_;

};
