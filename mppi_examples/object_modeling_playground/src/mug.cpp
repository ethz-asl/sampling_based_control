#include <cmath>
#include "object_modeling_playground/mug.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


Eigen::Vector3d Mug::get_pt_from_kpArray(int obj_idx, int pt_idx)
{
    // get the point3d from the ObjectsArray msgs
    // obj_idx: the idx of the object
    // pt_idx: the idx of the point in the object
    if(obj_idx > this->obj_num)
    {
        ROS_WARN_ONCE("Trying to get object outside the given ObjectArray.msg range");
    }

    if(pt_idx > keypoint_obj_array_msg->KeypointsArrays[obj_idx].PointSize)
    {
        ROS_WARN_ONCE("Trying to get keypoint outside the given KeypointsArray.msg range");
    }

    Eigen::Vector3d pt_eigen;
    geometry_msgs::Point pt_msg;
    bool pt_validity = keypoint_obj_array_msg->KeypointsArrays[obj_idx].keypoints[pt_idx].valid;

    if(pt_validity)
    {
        pt_msg = keypoint_obj_array_msg->KeypointsArrays[obj_idx].keypoints[pt_idx].point3d;
        pt_eigen << pt_msg.x, pt_msg.y, pt_msg.z;
    }
    else
    {
        //TODO: Don't know how to deal with invalide point now
    }

    return pt_eigen;

}
bool Mug::primitive_estimate(int obj_idx) 

{   // --------------------------------------------------------------------//
            // for Mug: [handle hole,   bottom,      top,     handle edge]
            // Input: idx of the keypointsArray from the msg
            // Updated: JointState state_ [ x,y,z, qx, qy, qz, qw] 
     // -----------------------------------------------------------------//
    
    // get point (Different for different object)
    Eigen::Vector3d kp_bottom = get_pt_from_kpArray(obj_idx, this->bottom_idx);
    Eigen::Vector3d kp_top    = get_pt_from_kpArray(obj_idx, this->top_idx);
    Eigen::Vector3d kp_handle = get_pt_from_kpArray(obj_idx, this->handle_idx);
    Eigen::Vector3d kp_avg    = get_pt_from_kpArray(obj_idx, this->avg_idx);

    //   GEOMETRY ESTIMATE
        // estimate the geometry feature
        // height and radius

    // center line (from bottow point to top)
    //center_line << keypoints_xd.segment(top_idx*4,3) - keypoints_xd.segment(bottom_idx*4, 3);
    center_line << kp_top - kp_bottom;

    // height  (= norm of the center line)
    mug_primitive.body_height = center_line.norm();

    // radius  ( = the dist between handle cirle point to the center line)
    // mug_primitive.body_radius = point_to_line(keypoints_xd.segment(bottom_idx*4,3),
    //                                           keypoints_xd.segment(top_idx*4,3),
    //                                           keypoints_xd.segment(avg_idx*4,3) );
    mug_primitive.body_radius = point_to_line(kp_bottom, kp_top, kp_avg);
                                                

    // POSE 
        // estimate the center point's pose(s) of the objects, will be used as the pose 
        // for the primitive(s)
    // ROS_INFO("pose estimation");
    // ROS_INFO_STREAM(keypoints_xd.transpose());
    state_.header.stamp = ros_time;
    state_.header.frame_id = prim_frame;
    state_.position.resize(primitive_num*7);
    state_.velocity.resize(primitive_num*7);
    Eigen::Vector3d center_pos;
    Eigen::Vector4d center_orien;

    estimate_center_pose(center_pos, center_orien, obj_idx);

    state_.position[0] = center_pos[0];
    state_.position[1] = center_pos[1];
    state_.position[2] = center_pos[2];
    
    state_.position[3] = center_orien[0]; //x
    state_.position[4] = center_orien[1]; //y
    state_.position[5] = center_orien[2]; //z
    state_.position[6] = center_orien[3]; //w

    //kptoPrimitive();   //TODO: comment this out for develop in local frame

    return true;
}

void Mug::kptoPrimitive()
{
    // 3d kp to primitive msg
    mug_primitive.pose.position.x = state_.position[0];
    mug_primitive.pose.position.y = state_.position[1];
    mug_primitive.pose.position.z = state_.position[2];

    mug_primitive.pose.orientation.x = state_.position[3];
    mug_primitive.pose.orientation.y = state_.position[4];
    mug_primitive.pose.orientation.z = state_.position[5];
    mug_primitive.pose.orientation.w = state_.position[6];


    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_(tf_buffer_);

    geometry_msgs::TransformStamped transform;

    geometry_msgs::PoseStamped pri_pos_in_pri;
    pri_pos_in_pri.header = mug_primitive.header;
    pri_pos_in_pri.pose.position = mug_primitive.pose.position;
    pri_pos_in_pri.pose.orientation = mug_primitive.pose.orientation;

    geometry_msgs::PoseStamped pri_pos_in_world;

    transform = tf_buffer_.lookupTransform("world", "camera_color_optical_frame",  ros::Time(0), ros::Duration(1.0) );

    tf2::doTransform(pri_pos_in_pri, pri_pos_in_world, transform); 
    // ROS_INFO_STREAM("transformed: " << pri_pos_in_world);


}
double Mug::point_to_line(const Eigen::Vector3d& pos_1,
                        const Eigen::Vector3d& pos_2,
                        const Eigen::Vector3d& pos_3){
    // cal distance between pos_3 and pos_1_2 line
    // the principle of computation can be found https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html

    auto nominator = (pos_3-pos_1).cross(pos_3-pos_2);
    auto denominator = pos_2 - pos_1;
    double sqrt_nom =  nominator.norm();
    double sqrt_denom = denominator.norm();

    // ROS_INFO_STREAM("radius is: " << (sqrt_nom/sqrt_denom));
    return (sqrt_nom/sqrt_denom);
}

Eigen::Matrix3d Mug::rot_of_two_frame(const Eigen::Matrix3d& rot_1,
                            const Eigen::Matrix3d& rot_2){
    // calculate the angle between two given vectors
    // double cos_angle = vec_1.dot(vec_2) / ( vec_1.norm() * vec_2.norm() );
    // double angle = std::acos(cos_angle);
    // ROS_INFO_STREAM("angle is: " << angle);
    return rot_1*rot_2;
}


bool Mug::estimate_center_pose(Eigen::Vector3d& pos,
                        Eigen::Vector4d& orien,
                        int obj_idx)
{   

    // get point (Different for different object)
    Eigen::Vector3d kp_bottom = get_pt_from_kpArray(obj_idx, this->bottom_idx);
    Eigen::Vector3d kp_top    = get_pt_from_kpArray(obj_idx, this->top_idx);
    Eigen::Vector3d kp_handle = get_pt_from_kpArray(obj_idx, this->handle_idx);
    Eigen::Vector3d kp_avg    = get_pt_from_kpArray(obj_idx, this->avg_idx);
    // TODO: this is reperated, fix this later


    // calculate a single primitve's center position & orientation

    // position
    // pos << center_line/2 + keypoints_xd.segment(bottom_idx*4, 3);
    pos << center_line/2 + kp_bottom;
    
    // orientation
        // roll  (= 0)
    center_roll = 0;
        // pitch (= 0)
    center_pitch = 0;
        // yaw  (= angle between center line and the camera_world_frame) 
    // TODO: figure out the frame of the camera_world_frame
    // for now, set this frame ideantical to ref_frame
    Eigen::Vector3d ref_x_axis = {1,0,0};
    Eigen::Vector3d ref_y_axis = {0,1,0};
    Eigen::Vector3d ref_z_axis = {0,0,1};
    Eigen::Matrix3d ref_rot; 
    ref_rot << 1,0,0, 
               0,1,0, 
               0,0,1;
    Eigen::Vector3d pri_x_axis,pri_y_axis,pri_z_axis;
    pri_z_axis = center_line.normalized();
    // pri_x_axis = (keypoints_xd.segment(handle_idx*4, 3) - pos).normalized(); //TODO: should be point-to-line
    pri_x_axis = (kp_handle - pos).normalized(); //TODO: should be point-to-line
    pri_y_axis = pri_z_axis.cross(pri_x_axis);
    Eigen::Matrix3d pri_rot; 
    pri_rot.col(0) = pri_x_axis;
    pri_rot.col(1) = pri_y_axis;
    pri_rot.col(2) = pri_z_axis;

    // center_yaw = rot_of_two_frame(ref_rot, pri_rot);
    
    Eigen::Matrix3d rot = rot_of_two_frame(ref_rot, pri_rot);
    Eigen::Quaterniond q(rot);
    q = q.normalized();

    orien[0] = q.x();
    orien[1] = q.y();
    orien[2] = q.z();
    orien[3] = q.w();

    return true;
} 

void Mug::update()
{   
    if(!kp_msg_received && !sim)
    {
        ROS_INFO("kp msg not received, waiting...");
        return;
    }
    if(sim)
        ros_time = ros::Time::now();

    for(int i = 0; i < this->obj_num; i ++ )
    {    
        primitive_estimate(i);  
        update_TF();
        primitive_visualize();
        pub_state();
    }

    ransac_fitting();

}

void Mug::ransac_fitting()
{   

    return;
}

Mug::Mug(const ros::NodeHandle& nh):nh_(nh),Object(nh)
{
    ROS_INFO("[Object: mug(ros::NodeHandle& nh)]");
    if(sim)
    {  
    }

    this->avg_idx = 0;
    this->handle_idx = 1;
    this->bottom_idx = 2;
    this->top_idx = 3;

    mug_primitive.header.frame_id = ref_frame;
    ROS_INFO_STREAM("primitive state in [ " << ref_frame << " ]");
}


void Mug::update_TF()
{   
    // update TF between object_frame to referrence_frame
    if(sim)
    {
        obj_trans.header.stamp = ros_time;
        obj_trans.transform.translation.x = 0.4;
        obj_trans.transform.translation.y = -0.4;
        obj_trans.transform.translation.z = 0.2;
        tf2::Quaternion q_;
        q_.setEuler(1.7,0,0.5);
        obj_trans.transform.rotation.x = q_.x();
        obj_trans.transform.rotation.y = q_.y();
        obj_trans.transform.rotation.z = q_.z();
        obj_trans.transform.rotation.w = q_.w();

        broadcaster.sendTransform(obj_trans);
    }


    // update TF between primitive_frame to referrence_frame
    prim_trans.header.stamp = ros_time;
    prim_trans.transform.translation.x = state_.position[0];
    prim_trans.transform.translation.y = state_.position[1];
    prim_trans.transform.translation.z = state_.position[2];

    prim_trans.transform.rotation.x = state_.position[3];
    prim_trans.transform.rotation.y = state_.position[4];
    prim_trans.transform.rotation.z = state_.position[5];
    prim_trans.transform.rotation.w = state_.position[6];
    broadcaster.sendTransform(prim_trans);
}




void Mug::primitive_visualize()
{   
    primitive_markers_.markers.clear();
    primitive_markers_.markers.resize(primitive_num);
    for(int i = 0 ; i < primitive_num; i++)
    {   
        primitive_marker_.type = visualization_msgs::Marker::CYLINDER;
        primitive_marker_.id = i;
        primitive_marker_.header.frame_id = prim_frame;
        primitive_marker_.header.stamp = ros_time;
        primitive_marker_.action = visualization_msgs::Marker::ADD;
        primitive_marker_.color.r = 0.0;
        primitive_marker_.color.b = 0.0;
        primitive_marker_.color.g = 1.0;
        primitive_marker_.color.a = 0.8;
        primitive_marker_.scale.x = 2*mug_primitive.body_radius;
        primitive_marker_.scale.y = 2*mug_primitive.body_radius;
        primitive_marker_.scale.z = mug_primitive.body_height;
        primitive_marker_.pose.orientation.x = 0;
        primitive_marker_.pose.orientation.y = 0;
        primitive_marker_.pose.orientation.z = 0;
        primitive_marker_.pose.orientation.w = 1;
        primitive_markers_.markers[i] = primitive_marker_;
    }
}

void Mug::pub_state()
{
    state_publisher_.publish(mug_primitive);
    primitive_publisher_.publish(primitive_markers_);
}