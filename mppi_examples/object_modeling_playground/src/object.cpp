
#include <cmath>
#include "object_modeling_playground/object.h"


void Object::kp_int_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{   
    // save the current kp for state estiamtion
    keypoints_past_ = keypoints_;
    kp_num = msg->poses.size();
    keypoints_.resize(kp_num*4);
    
    keypoints_xd.resize(kp_num*4);

    for (int i = 0 ; i < kp_num; i++)
    { 
        keypoints_xd.segment(i*4,3) << 
        msg->poses[i].position.x,msg->poses[i].position.y,msg->poses[i].position.z;
        keypoints_xd(i*4 + 3) = 0 ;
        keypoints_[i*4+1] = msg->poses[i].position.x;
        keypoints_[i*4+2] = msg->poses[i].position.y;
        keypoints_[i*4+3] = msg->poses[i].position.z;
    }
    pose_estimate();  
}


Object::Object(const ros::NodeHandle& nh):nh_(nh)
{
    init_param();

    state_publisher_=nh_.advertise<sensor_msgs::JointState>("/"+object_name+"/joint_states",10);   
    if(sim)
    {
      ROS_INFO("loading simulated gt object");
      kp_publisher_=nh_.advertise<visualization_msgs::MarkerArray>("/"+object_name+"/keypoints_marker_array",10);
      this->kp_trans.header.frame_id = "world";
      this->kp_trans.child_frame_id = "kp_frame";
      init_kp_TF();
    }
    primitive_publisher_=nh_.advertise<visualization_msgs::MarkerArray>("/"+object_name+"/primitives_marker_array",1000);
    
    kp_int_subscriber_ = 
        nh_.subscribe(kp_3d_topic, 100, &Object::kp_int_callback, this);
    ROS_INFO_STREAM(" subscribe keypoints 3d poses" << kp_3d_topic);
    this->obj_trans.header.frame_id = "world";
    this->obj_trans.child_frame_id = ref_frame;

}

bool Object::init_param()
{   
    std::vector<double> obj_scale;
    std::vector<double> obj_pos;
    std::vector<double> obj_rot;
    std::vector<double> keypoints;

    if (!nh_.getParam("object/object_name", object_name)) {
    ROS_ERROR("Failed to parse /object/object_name or invalid!");
    return false;
  }  

    if (!nh_.getParam("object/object_frame", ref_frame)) {
    ROS_ERROR("Failed to parse /object/object_frame or invalid!");
    return false;
  }  

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

    if (!nh_.getParam("approx_params/primitive_numbers", primitive_num)) {
    ROS_ERROR("Failed to parse approx_params/primitive_numbers or invalid!");
    return false;
  }    
    if (!nh_.getParam("approx_params/bottom_ind", bottom_ind)) {
    ROS_ERROR("Failed to parse approx_params/bottom_ind or invalid!");
    return false;
  }  

  if (!nh_.getParam("kp_3d_topic", kp_3d_topic)) {
    ROS_ERROR("Failed to parse kp_3d_topic or invalid!");
    return false;
  }  

    if (!nh_.getParam("sim", sim)) {
    ROS_ERROR("Failed to parse /sim or invalid!");
    return false;
  }  
    obj_scale_ = obj_scale;
    obj_pos_ = obj_pos;
    obj_rot_ = obj_rot;
    
    if(sim)
    {
      keypoints_ = keypoints;
      kp_num = keypoints_.size()/4;
      height.resize(primitive_num);
      radius.resize(primitive_num);
    }

    ROS_INFO_STREAM("Object Params [" << object_name << "] passed successfully ");
    return true;
}

void Object::init_kp_TF()
{
    kp_trans.header.stamp = ros::Time::now();
    kp_trans.transform.translation.x = obj_pos_[0];
    kp_trans.transform.translation.y = obj_pos_[1];
    kp_trans.transform.translation.z = obj_pos_[2];
    tf2::Quaternion q_;
    q_.setRPY(obj_rot_[0], obj_rot_[1], obj_rot_[2]);
    kp_trans.transform.rotation.x = q_.x();
    kp_trans.transform.rotation.y = q_.y();
    kp_trans.transform.rotation.z = q_.z();
    kp_trans.transform.rotation.w = q_.w();
}

