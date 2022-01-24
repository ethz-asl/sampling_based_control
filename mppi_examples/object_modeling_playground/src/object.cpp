
#include <cmath>
#include "object_modeling_playground/object.h"

void Object::kp_int_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{   

    // save the current kp for state estiamtion
    kp_num = msg->poses.size();
    // ROS_INFO_STREAM("kp num: "<< kp_num);
    ros_time = msg->header.stamp;

    if(kp_num == 0)
    { 
      ROS_INFO_STREAM(" No detection ");
      // TODO: no detection -> action
      return;
    }

    // filtering
    double alpha = 0.5;
    keypoints_filter(msg, alpha);

    keypoints_xd = keypoints_filtered.segment( (filter_length-1)*4*gt_kp_num, 4*gt_kp_num );

    kp_msg_received = true;
    
}

void Object::keypoints_filter(const geometry_msgs::PoseArray::ConstPtr& msg, double alpha)
{
    kp_num = msg->poses.size();
    Eigen::VectorXd keypoints_temp;
    keypoints_temp.setZero(gt_kp_num*4);

    for (int i = 0 ; i < gt_kp_num; i++)
    {   

        if(msg->poses[i].position.z < -90)  // if a invalid point, keep the last detected state
        {
          keypoints_temp.segment(i*4,4) = keypoints_filtered.segment(i*4,4);
        }
        else
        { 
          // ROS_INFO_STREAM(11);
          // ROS_INFO_STREAM("filtered "<<keypoints_filtered);
          // keypoints_filtered(i*4) = alpha*(msg->poses[i].position.x) + (1-alpha)*keypoints_filtered(i*4);
          // keypoints_filtered(i*4+1) = alpha*(msg->poses[i].position.y) + (1-alpha)*keypoints_filtered(i*4+1);
          // keypoints_filtered(i*4+2) = alpha*(msg->poses[i].position.z) + (1-alpha)*keypoints_filtered(i*4+2);
          // keypoints_filtered.segment(i*4,3) <<
          // alpha*msg->poses[i].position.x + (1-alpha)*keypoints_filtered[i*4],
          // alpha*msg->poses[i].position.y + (1-alpha)*keypoints_filtered[i*4 +1],
          // alpha*msg->poses[i].position.z + (1-alpha)*keypoints_filtered[i*4 +2];

          keypoints_temp.segment(i*4,3) <<
          msg->poses[i].position.x,
          msg->poses[i].position.y,
          msg->poses[i].position.z;

          // ROS_INFO_STREAM(22);
          // keypoints_filtered(i*4 + 3) = 0 ;
          keypoints_temp(i*4+3) =0;
        }
    }


    for(int i = 0 ; i < filter_length -1 ; i++)
    {
      keypoints_filtered.segment(i*4*gt_kp_num,4*gt_kp_num) =  
          keypoints_filtered.segment((i+1)*4*gt_kp_num,4*gt_kp_num);
    }

    keypoints_filtered.segment( (filter_length-1)*4*gt_kp_num, 4*gt_kp_num ).setZero();

    for(int i = 0 ; i < filter_length -1 ; i++)
    {
      keypoints_filtered.segment( (filter_length-1)*4*gt_kp_num, 4*gt_kp_num ) += 
          filter_alpha[i]*keypoints_filtered.segment(i*4*gt_kp_num, 4*gt_kp_num);
    }

    keypoints_filtered.segment( (filter_length-1)*4*gt_kp_num, 4*gt_kp_num ) += 
            filter_alpha[filter_length-1]*keypoints_temp;
    // ROS_INFO_STREAM("kp filtered final " << keypoints_filtered.transpose());
}

Object::Object(const ros::NodeHandle& nh):nh_(nh)
{
    init_param();

    state_publisher_=nh_.advertise<manipulation_msgs::MugPrimitive>("/"+object_name+"/joint_states",10);   

    kp_publisher_=nh_.advertise<visualization_msgs::MarkerArray>("/"+object_name+"/keypoints_marker_array",10);
    kp_markers_.markers.clear();
    kp_markers_.markers.resize(gt_kp_num);

    for(int i = 0 ; i<gt_kp_num; i++)
    {   
        kp_marker_.type = visualization_msgs::Marker::SPHERE;
        kp_marker_.id = i;
        if(sim)
          kp_marker_.header.frame_id = obj_frame;
        else if(!sim)
          kp_marker_.header.frame_id = ref_frame;
        kp_marker_.action = visualization_msgs::Marker::ADD;
        kp_marker_.color.r = 0.6;
        kp_marker_.color.b = 0.0;
        kp_marker_.color.g = 0.4;
        kp_marker_.color.a = 1.0;
        kp_marker_.scale.x = 0.012;
        kp_marker_.scale.y = 0.012;
        kp_marker_.scale.z = 0.012;
        kp_marker_.pose.orientation.x = 0.0;
        kp_marker_.pose.orientation.y = 0.0;
        kp_marker_.pose.orientation.z = 0.0;
        kp_marker_.pose.orientation.w = 1.0;
        kp_marker_.pose.position.z = 0;
        kp_marker_.pose.position.y = 0;
        kp_marker_.pose.position.x = 0;
        kp_markers_.markers[i] = kp_marker_;
    }
    ROS_INFO("[keypoints marker inited");
    
    primitive_publisher_=nh_.advertise<visualization_msgs::MarkerArray>("/"+object_name+"/primitives_marker_array",1000);
    kp_int_subscriber_ = 
        nh_.subscribe(kp_3d_topic, 100, &Object::kp_int_callback, this);
    ROS_INFO_STREAM(" subscribe keypoints 3d poses" << kp_3d_topic);
    obj_trans.header.frame_id = ref_frame;
    obj_trans.child_frame_id = obj_frame;

    if(!sim)
      prim_trans.header.frame_id = ref_frame; // TODO: make it also in ref_frame
    else if(sim)
    {
      prim_trans.header.frame_id = obj_frame;
    }

    prim_trans.child_frame_id = prim_frame;

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

    if (!nh_.getParam("object/object_frame", obj_frame)) {
    ROS_ERROR("Failed to parse /object/object_frame or invalid!");
    return false;
  }  

    if (!nh_.getParam("object/ref_frame", ref_frame)) {
    ROS_ERROR("Failed to parse /object/ref_frame or invalid!");
    return false;
  }
      
    if (!nh_.getParam("object/primitive_frame", prim_frame)) {
    ROS_ERROR("Failed to parse /object/primitive_frame or invalid!");
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

  if (!nh_.getParam("kp_3d_topic", kp_3d_topic)) {
    ROS_ERROR("Failed to parse kp_3d_topic or invalid!");
    return false;
  }  

  if (!nh_.getParam("sim", sim)) {
    ROS_ERROR("Failed to parse sim or invalid!");
    return false;
  } 

  if (!nh_.getParam("filter/alpha", filter_alpha)) {
    ROS_ERROR("Failed to parse filter/alpha or invalid!");
    return false;
  } 


  ROS_INFO_STREAM("Object Params [" << object_name << "] passed successfully ");
  obj_scale_ = obj_scale;
  obj_pos_ = obj_pos;
  obj_rot_ = obj_rot;
  
  gt_kp_num = int(keypoints.size()/4); //TODO: make this read from keypoint.json

  filter_length = filter_alpha.size();
  keypoints_xd.setZero(gt_kp_num*4);
  keypoints_filtered.setZero(filter_length*gt_kp_num*4);

  if(sim)
  { 
    ROS_INFO("In simulation mode, use gt object");
    keypoints_sim_xd.resize(keypoints.size());
    for(int i=0; i<keypoints.size(); i ++)
    keypoints_sim_xd[i] = keypoints[i];
    
    // in simulation, TODO: use the gt kp and transform into the point in ref_frame
    keypoints_xd = keypoints_sim_xd;
    ROS_INFO_STREAM("keypoinys of [" << object_name << "] init successfully, # of points = " << gt_kp_num);
  }

  
    return true;
}
