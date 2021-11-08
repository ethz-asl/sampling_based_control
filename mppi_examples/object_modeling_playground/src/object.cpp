#include "object.h"


bool Object::init_param()
{   
    std::vector<double> obj_scale;
    std::vector<double> obj_pos;
    std::vector<double> obj_rot;
    std::vector<double> keypoints;
    if (!nh_.getParam("object/scale", obj_scale)) {
    ROS_ERROR("Failed to parse /object/scale or invalid!");
    return false;
  }  
    if (!nh_.getParam("object/position", obj_pos)) {
    ROS_ERROR("Failed to parse /object/position or invalid!");
    return false;
  }  
    if (!nh_.getParam("object/orientation", obj_rot)) {
    ROS_ERROR("Failed to parse /object/orientation or invalid!");
    return false;
  }    
    if (!nh_.getParam("keypoints", keypoints)) {
    ROS_ERROR("Failed to parse /keypoints or invalid!");
    return false;
  }    

    obj_scale_ = obj_scale;
    obj_pos_ = obj_pos;
    obj_rot_ = obj_rot;
    keypoints_ = keypoints;

    // for (int i = 0 ; i < obj_scale_.size(); i++)
    // {
    //     std::cout << obj_scale_[i] << std::endl;
    // }   
    // nh.setParam("/global_param", 5);

    return true;
}

void Object::setTF()
{
    state_.header.stamp = ros::Time::now();
    trans.header.stamp = ros::Time::now();
    trans.transform.translation.x = obj_pos_[0];
    trans.transform.translation.y = obj_pos_[1];
    trans.transform.translation.z = obj_pos_[2];
    tf2::Quaternion q_;
    q_.setRPY(obj_rot_[0], obj_rot_[1], obj_rot_[2]);
    trans.transform.rotation.x = q_.x();
    trans.transform.rotation.y = q_.y();
    trans.transform.rotation.z = q_.z();
    trans.transform.rotation.w = q_.w();
}

void Object::init_object_publisher(std::string topic_name, int rate)
{
    state_publisher_=nh_.advertise<sensor_msgs::JointState>(topic_name,rate);   
}

void Object::init_kp_array_publisher(std::string topic_name, int rate)
{
    kp_publisher_=nh_.advertise<visualization_msgs::MarkerArray>(topic_name,rate);
}

void Object::create_kp_markers(std::string ref_frame)
{   
    kp_num = keypoints_.size()/4;
    // handle
    for(int i = 0 ; i<kp_num; i++)
    {
        kp_marker_.type = visualization_msgs::Marker::SPHERE;
        kp_marker_.id = keypoints_[i*4];
        kp_marker_.header.frame_id = ref_frame;
        kp_marker_.action = visualization_msgs::Marker::ADD;
        kp_marker_.color.r = 1.0;
        kp_marker_.color.b = 0.0;
        kp_marker_.color.g = 0.0;
        kp_marker_.color.a = 1.0;
        kp_marker_.scale.x = 0.015;
        kp_marker_.scale.y = 0.015;
        kp_marker_.scale.z = 0.015;
        kp_marker_.pose.orientation.x = 0.0;
        kp_marker_.pose.orientation.y = 0.0;
        kp_marker_.pose.orientation.z = 0.0;
        kp_marker_.pose.orientation.w = 1.0;
        kp_marker_.pose.position.z = keypoints_[i*4+3];
        kp_marker_.pose.position.y = keypoints_[i*4+2];
        kp_marker_.pose.position.x = keypoints_[i*4+1];
        kp_markers_.markers.push_back(kp_marker_);
    }

}

void Object::fit_primitive()
{
    std::vector<double> z_pos;
    z_pos.clear();
    for (int i = 0 ; i < kp_num; i++ )
    {
        z_pos.push_back(keypoints_[i*4+3]);
        std::cout << "z pos: " << keypoints_[i*4+3]<<std::endl;
    }
    int max_z_index = std::max_element(z_pos.begin(),z_pos.end()) - z_pos.begin();
    double max_z = *std::max_element(z_pos.begin(), z_pos.end());
    int min_z_index = std::min_element(z_pos.begin(),z_pos.end()) - z_pos.begin();
    double min_z = *std::min_element(z_pos.begin(), z_pos.end());

    //std::cout << "max z : " << max_z << std::endl;
    //std::cout << "min z : " << min_z << std::endl;

    double height = max_z - min_z;

}

void Object::pub_state()
{
    state_publisher_.publish(state_);
    kp_publisher_.publish(kp_markers_);
}