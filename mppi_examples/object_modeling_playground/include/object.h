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

    bool init_param();
    void init_object_publisher(std::string topic_name, int rate);
    void setTF();
    void pub_state();
    void init_kp_array_publisher(std::string topic_name, int rate);
    void init_primitive_array_publisher(std::string topic_name, int rate);
    void update_kp_markers(std::string ref_frame);
    void fit_primitive();
    void vis_primitive();

private:
    int kp_num;
    std::vector<int> set_nums;
    int bottom_ind;
    std::string ref_frame;

    // params of approx primitive
    int primitive_num = 0 ;
    std::vector<double> height;
    std::vector<double> radius;

    ros::NodeHandle nh_;

    ros::Publisher state_publisher_;
    sensor_msgs::JointState state_; 

    ros::Publisher kp_publisher_;
    visualization_msgs::MarkerArray kp_markers_;
    visualization_msgs::Marker kp_marker_;

    ros::Publisher primitive_publisher_;
    visualization_msgs::MarkerArray primitive_markers_;
    visualization_msgs::Marker primitive_marker_;

    std::vector<double> obj_scale_;
    std::vector<double> obj_pos_;
    std::vector<double> obj_rot_;
    std::vector<double> keypoints_;



public:

    geometry_msgs::TransformStamped trans;
};