#pragma once
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <chrono>
#include <Eigen/Core>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/transform_listener.h>
#include <manipulation_msgs/MugPrimitive.h>
#include <keypoint_msgs/KeypointsArray.h>
#include <keypoint_msgs/keypoint.h>
#include <keypoint_msgs/ObjectsArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/filters/voxel_grid.h>


class Object
{
public:
    Object(const ros::NodeHandle& nh);
    ~Object() = default;
    virtual void update_TF(){};
    virtual void pub_state(){};
    virtual void primitive_visualize(){};
    virtual bool primitive_estimate(int obj_idx){};
    virtual void update(){};
    
    geometry_msgs::TransformStamped obj_trans;
    geometry_msgs::TransformStamped prim_trans;
    geometry_msgs::TransformStamped kp_trans;
    
protected:
    bool init_param();
    void kp_msg_callback(const keypoint_msgs::ObjectsArray::ConstPtr& msg);
    void pcl_roi_callback(const sensor_msgs::PointCloud2::ConstPtr& pcl_msg);
    pcl::PCLPointCloud2::Ptr pcl_downsampling(pcl::PCLPointCloud2::Ptr dense_pcl_ptr);  
    void keypoints_filter(const geometry_msgs::PoseArray::ConstPtr& msg, double alpha);

    bool sim;
    std::string kp_msg_topic;
    std::string pcl_msg_topic;

    // obj config vars
    std::string ref_frame;
    std::string obj_frame;
    std::string prim_frame;
    std::string object_name;
    std::vector<double> obj_scale_;
    std::vector<double> obj_pos_;
    std::vector<double> obj_rot_;
    int primitive_num = 0 ;
    
    // filter
    int filter_length;
    std::vector<double> filter_alpha;

    // kp vars
    int kp_num;
    int obj_num;
    int gt_kp_num;
    Eigen::VectorXd keypoints_xd;
    Eigen::VectorXd keypoints_filtered;
    Eigen::VectorXd keypoints_sim_xd;
    std::vector<double> keypoints_;
    std::vector<double> keypoints_past_;

    // ros vars
    ros::Subscriber kp_msg_subscriber_;
    ros::Subscriber pcl_msg_subscriber_;
    ros::Publisher state_publisher_;
    ros::Publisher primitive_publisher_;

    sensor_msgs::JointState state_; 
    visualization_msgs::MarkerArray primitive_markers_;
    visualization_msgs::Marker primitive_marker_;
    
    keypoint_msgs::ObjectsArray::ConstPtr keypoint_obj_array_msg;
    sensor_msgs::PointCloud2::ConstPtr pcl_ptr;

    tf2_ros::TransformBroadcaster broadcaster;


    bool kp_msg_received = false;
    ros::Time ros_time;
    std::string kp_ref_frame;

    // PCL
    //pcl::PCLPointCloud2::Ptr pcl_pc2_ptr;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloudXYZ;
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

private:
    // params of approx primitive
    ros::NodeHandle nh_;

};
