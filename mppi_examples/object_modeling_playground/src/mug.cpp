#include <cmath>
#include "object_modeling_playground/mug.h"



void Mug::primitive_estimate() 

{   // --------------------------------------------------------------------//
            // for Mug: [handle hole,   bottom,      top,     handle edge]
            // Input: Eigen::VectorXd keypoints_xd,  size(kp_num*4)
            // Updated: JointState state_ [ x,y,z, qx, qy, qz, qw] 
     // -----------------------------------------------------------------//
    
    //   GEOMETRY ESTIMATE
        // estimate the geometry feature
        // height and radius

    // center line (from bottow point to top)
    center_line << keypoints_xd.segment(2*4,3) - keypoints_xd.segment(1*4, 3);

    // height  (= norm of the center line)
    mug_primitive.body_height = center_line.norm();

    // radius  ( = the dist between handle cirle point to the center line)
    mug_primitive.body_radius = point_to_line(keypoints_xd.segment(1*4,3),
                                              keypoints_xd.segment(2*4,3),
                                              keypoints_xd.segment(0*4,3) );

    // POSE 
        // estimate the center point's pose(s) of the objects, will be used as the pose 
        // for the primitive(s)
    ROS_INFO("pose estimation");
    ROS_INFO_STREAM(keypoints_xd.transpose());
    state_.header.stamp = ros_time;
    state_.header.frame_id = prim_frame;
    state_.position.resize(primitive_num*7);
    state_.velocity.resize(primitive_num*7);
    Eigen::Vector3d center_pos;
    Eigen::Vector4d center_orien;
    estimate_center_pose(center_pos, center_orien);

    state_.position[0] = center_pos[0];
    state_.position[1] = center_pos[1];
    state_.position[2] = center_pos[2];
    
    state_.position[3] = center_orien[0]; //x
    state_.position[4] = center_orien[1]; //y
    state_.position[5] = center_orien[2]; //z
    state_.position[6] = center_orien[3]; //w


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

    ROS_INFO_STREAM("radius is: " << (sqrt_nom/sqrt_denom));
    return (sqrt_nom/sqrt_denom);
}

double Mug::angle_of_two_vector(const Eigen::Vector3d& vec_1,
                            const Eigen::Vector3d& vec_2){
    // calculate the angle between two given vectors
    double cos_angle = vec_1.dot(vec_2) / ( vec_1.norm() * vec_2.norm() );
    double angle = std::acos(cos_angle);
    ROS_INFO_STREAM("angle is: " << angle);
    return angle;
}


bool Mug::estimate_center_pose(Eigen::Vector3d& pos,
                        Eigen::Vector4d& orien)
{
    // calculate a single primitve's center position & orientation

    // position
    pos << center_line/2 + keypoints_xd.segment(1*4, 3);
    
    // orientation
        // roll  (= 0)
    center_roll = 0;
        // pitch (= 0)
    center_pitch = 0;
        // yaw  (= angle between center line and the camera_world_frame) 
    // TODO: figure out the frame of the camera_world_frame
    // for now, set this frame ideantical to ref_frame
    Eigen::Vector3d ref_z_axis = {0,0,1};
    center_yaw = angle_of_two_vector(center_line, ref_z_axis);
    tf2::Quaternion q_;
    // q_.setRPY(center_roll,center_pitch,center_yaw);
    q_.setEuler(center_yaw,center_pitch,center_roll);
    orien[0] = q_.x();
    orien[1] = q_.y();
    orien[2] = q_.z();
    orien[3] = q_.w();

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
    primitive_estimate();  
    update_kp_markers();
    update_TF();
    primitive_visualize();
    pub_state();
}

Mug::Mug(const ros::NodeHandle& nh):nh_(nh),Object(nh)
{

    if(sim)
    {  
    }

    ROS_INFO("[Object: mug(ros::NodeHandle& nh)]");
}


void Mug::update_TF()
{   
    // update TF between object_frame to referrence_frame
    if(sim)
    {
        obj_trans.header.stamp = ros_time;
        obj_trans.transform.translation.x = 1;
        obj_trans.transform.translation.y = 1;
        obj_trans.transform.translation.z = 1;
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



void Mug::update_kp_markers()
{   
    for(int i = 0 ; i<kp_num; i++)
    {   
        kp_markers_.markers[i].header.stamp = ros_time;
        if(sim)
        {
            kp_markers_.markers[i].pose.position.z = keypoints_sim_xd[i*4+2];
            kp_markers_.markers[i].pose.position.y = keypoints_sim_xd[i*4+1];
            kp_markers_.markers[i].pose.position.x = keypoints_sim_xd[i*4+0];
        }
        if((!sim) && kp_msg_received)
        {   
            kp_markers_.markers[i].pose.position.z = keypoints_xd[i*4+2];
            kp_markers_.markers[i].pose.position.y = keypoints_xd[i*4+1];
            kp_markers_.markers[i].pose.position.x = keypoints_xd[i*4+0];
        }
    }
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
    state_publisher_.publish(state_);

    kp_publisher_.publish(kp_markers_);
    primitive_publisher_.publish(primitive_markers_);
}