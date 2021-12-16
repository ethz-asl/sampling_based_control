#include <cmath>
#include "object_modeling_playground/mug.h"


void Mug::pose_estimate() 
{
    // estimate T for TX = Y,  X is the kp of last frame and Y is kp of this frame
    ROS_INFO("pose estimation");
    ROS_INFO_STREAM(keypoints_xd.transpose());
}


Mug::Mug(const ros::NodeHandle& nh):nh_(nh),Object(nh)
{
    ROS_INFO("[Object: mug(ros::NodeHandle& nh)]");
}


void Mug::update_obj_TF()
{
    state_.header.stamp = ros::Time::now();
    kp_trans.header.stamp = ros::Time::now();
    obj_trans.header.stamp = ros::Time::now();
    obj_trans.transform.translation.x = obj_pos_[0];
    obj_trans.transform.translation.y = obj_pos_[1];
    obj_trans.transform.translation.z = obj_pos_[2];
    tf2::Quaternion q_;
    q_.setRPY(obj_rot_[0], obj_rot_[1], obj_rot_[2]);
    obj_trans.transform.rotation.x = q_.x();
    obj_trans.transform.rotation.y = q_.y();
    obj_trans.transform.rotation.z = q_.z();
    obj_trans.transform.rotation.w = q_.w();
}


void Mug::update_kp_markers()
{   
    kp_num = keypoints_.size()/4;
    set_nums.clear();
    set_nums.resize(kp_num,0);
    kp_markers_.markers.clear();
    kp_markers_.markers.resize(kp_num);

    for(int i = 0 ; i<kp_num; i++)
    {   
        kp_marker_.type = visualization_msgs::Marker::SPHERE;
        kp_marker_.id = i;
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
        kp_markers_.markers[i]= kp_marker_;

        set_nums[keypoints_[i*4]] +=1;
    }
}


void Mug::fit_primitive()
{
    // fit mug body
    double z_top_mean = 0.0;
    double z_bottom_mean = 0.0;
    double radius_mean = 0.0;
    height.clear();
    radius.clear();
    height.resize(primitive_num,0.0);
    radius.resize(primitive_num,0.0);
    for (int i = 0 ; i < kp_num; i++ )
    {   
        if (keypoints_[i*4] == 1)
        {
            height[0] += keypoints_[i*4+3] - keypoints_[bottom_ind*4+3];
            radius[0] += sqrt(pow(keypoints_[i*4+2]-keypoints_[bottom_ind*4+2],2)
                                +pow(keypoints_[i*4+1]-keypoints_[bottom_ind*4+1],2)); 
        }
        if (keypoints_[i*4] == 0){
            radius[0] += sqrt(pow(keypoints_[i*4+2]-keypoints_[bottom_ind*4+2],2)
                                +pow(keypoints_[i*4+1]-keypoints_[bottom_ind*4+1],2)); 
            //height[1] += keypoints_[i*4+3] - keypoints_[bottom_ind*4+3];
        }

    }

    // todo: switch to multi primitives
    height[0] = height[0]/set_nums[1];
    radius[0] = radius[0]/set_nums[1];

    // // fit mug handle 
    // height[1] = height[1]
   
}

void Mug::vis_primitive()
{   
    primitive_markers_.markers.clear();
    primitive_markers_.markers.resize(primitive_num);
    for(int i = 0 ; i < primitive_num; i++)
    {   
        primitive_marker_.type = visualization_msgs::Marker::CYLINDER;
        primitive_marker_.id = 1;
        primitive_marker_.header.frame_id = "mug_frame";
        primitive_marker_.header.stamp = ros::Time::now();
        primitive_marker_.action = visualization_msgs::Marker::ADD;
        primitive_marker_.color.r = 0.0;
        primitive_marker_.color.b = 0.0;
        primitive_marker_.color.g = 1.0;
        primitive_marker_.color.a = 0.8;
        primitive_marker_.scale.x = radius[0];
        primitive_marker_.scale.y = radius[0];
        primitive_marker_.scale.z = height[0];
        primitive_marker_.pose.orientation.x = 0.0;
        primitive_marker_.pose.orientation.y = 0.0;
        primitive_marker_.pose.orientation.z = 0.0;
        primitive_marker_.pose.orientation.w = 1.0;
        primitive_marker_.pose.position.z = keypoints_[bottom_ind*4+3] + height[i]/2;
        primitive_marker_.pose.position.y = keypoints_[bottom_ind*4+2];
        primitive_marker_.pose.position.x = keypoints_[bottom_ind*4+1];
        primitive_markers_.markers[i] = primitive_marker_;
    }
}

void Mug::pub_state()
{
    //state_publisher_.publish(state_);
    kp_publisher_.publish(kp_markers_);
    primitive_publisher_.publish(primitive_markers_);
}