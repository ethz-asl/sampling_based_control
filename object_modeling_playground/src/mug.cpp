#include <cmath>
#include "object_modeling_playground/mug.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

Eigen::Vector3d Mug::get_pt_from_kpArray(int obj_idx, int pt_idx)
{
    /* 
    Get the point3d from the ObjectsArray msgs
        obj_idx: the idx of the object
        pt_idx: the idx of the point in the object 
    */
    Eigen::Vector3d pt_eigen;
    geometry_msgs::Point pt_msg;

    if(obj_idx > this->obj_num)
    {
        ROS_WARN_ONCE("Trying to get object outside the given ObjectArray.msg range");
        return pt_eigen;
    }

    if(pt_idx > keypoint_obj_array_msg->KeypointsArrays[obj_idx].PointSize)
    {
        ROS_WARN_ONCE("Trying to get keypoint outside the given KeypointsArray.msg range");
        return pt_eigen;
    }

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

    // center line (from bottow point to top)
    center_line << kp_top - kp_bottom;

    //  GEOMETRY ESTIMATE
        // estimate the geometry feature
        // height and radius

    // height  (= norm of the center line)
    mug_primitive.body_height = center_line.norm();

    // radius  ( = the dist between handle cirle point to the center line)
    mug_primitive.body_radius = point_to_line(kp_bottom, kp_top, kp_avg);
                                                
    // POSE ESTIMATE
        // estimate the center point's pose(s) of the objects, will be used as the pose 
        // for the primitive(s)

    state_.header.stamp = ros_time;
    state_.header.frame_id = prim_frame;
    state_.position.resize(primitive_num*7);
    state_.velocity.resize(primitive_num*7);

    Eigen::Vector3d center_pos;
    Eigen::Vector4d center_orien;
    estimate_center_pose(center_pos, center_orien, obj_idx);

    // update primitive frame params, 
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

    transform = tf_buffer_.lookupTransform(world_frame, ref_frame,  ros::Time(0), ros::Duration(1.0) );

    tf2::doTransform(pri_pos_in_pri, pri_pos_in_world, transform); 
    ROS_INFO_STREAM("transformed: " << pri_pos_in_world);
    mug_primitive.pose.position = pri_pos_in_world.pose.position;
    mug_primitive.pose.orientation = pri_pos_in_world.pose.orientation;


}
double Mug::point_to_line(const Eigen::Vector3d& pos_1,
                        const Eigen::Vector3d& pos_2,
                        const Eigen::Vector3d& pos_3){
    // cal distance between pos_3 and pos_1_2 line
    // the principle of computation can be found https://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html

    auto nominator = (pos_3-pos_1).cross(pos_3-pos_2);
    auto denominator = pos_2 - pos_1;

    return (nominator.norm()/denominator.norm());
}

Eigen::Matrix3d Mug::rot_of_two_frame(const Eigen::Matrix3d& rot_1,
                            const Eigen::Matrix3d& rot_2){
    return rot_1*rot_2;
}


void Mug::get_orien_from_axisXZ(Eigen::Vector3d& axis_x, Eigen::Vector3d& axis_z, Eigen::Quaterniond& quat)
{   
    axis_x = axis_x.normalized();
    axis_z = axis_z.normalized();

    Eigen::Matrix3d ref_rot; 
    ref_rot << 1,0,0, 
               0,1,0, 
               0,0,1;
    Eigen::Vector3d axis_y = axis_z.cross(axis_x);
    Eigen::Matrix3d pri_rot;
    pri_rot.col(0) = axis_x;
    pri_rot.col(1) = axis_y;
    pri_rot.col(2) = axis_z;

    Eigen::Matrix3d rot = rot_of_two_frame(ref_rot, pri_rot);
    Eigen::Quaterniond q(rot);
    quat = q.normalized();

    return;
}


// TODO:: remove pos and orien arg, use obj_idx to read the pos and orien from a vector or Eigen
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
    pos << center_line/2 + kp_bottom;
    
    // orientation
        // the frame of the camera_world_frame is the ref_frame in this project with Realsense D
        // for now, set this frame ideantical to ref_frame

    Eigen::Vector3d pri_x_axis,pri_y_axis,pri_z_axis;
    pri_z_axis = center_line;
    pri_x_axis = kp_handle - pos; //TODO: should be point-to-line

    Eigen::Quaterniond q;
    get_orien_from_axisXZ(pri_x_axis,pri_z_axis,q);

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
        auto start = std::chrono::high_resolution_clock::now(); 
        //get3Dbbox_frompointcloud(i);
        ransac_fitting(i);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        //ROS_INFO_STREAM("ransac takes: " <<duration.count() << "  micro secs");
        update_TF();
        primitive_visualize();
        pub_state();
        kptoPrimitive();
    }

}

void Mug::get3Dbbox_frompointcloud(int obj_idx)
{   

    float radius = 0.1;
    typedef pcl::PointXYZ PointT;

    while (true)
    {           
        std::cout << " 0 " << std::endl;
        if(pcl_cloudXYZ.points.size() == 0 || !kp_msg_received)
        {
            continue;
        }
        
        std::cout << " 1 " << std::endl;
        pcl::PointCloud<PointT>::Ptr input_cloud(new pcl::PointCloud<PointT>);       
        std::vector<int> pointIdxRadiusSearch; //to store index of surrounding points 
        std::vector<float> pointRadiusSquaredDistance; // to store distance to surrounding points
        pcl::PointCloud<pcl::PointXYZ> pcl_could_3Dbbox_;

        auto centerPointIdx = this->avg_idx;
        Eigen::Vector3d kp_avg = get_pt_from_kpArray(obj_idx, this->avg_idx);  //TODO: add extra process to deal the missing kp situtations

        // // PCL search for pt in a ball area
        *input_cloud  = pcl_cloudXYZ;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(input_cloud);
        pcl::PointXYZ searchPoint(kp_avg[0],kp_avg[1],kp_avg[2]);
        kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance,2000);  // take 2000 from roughly 8000
   
        if ( kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {   
            pcl_could_3Dbbox_.header.frame_id = pcl_ref_frame;
            pcl_could_3Dbbox_.header.stamp = pcl_time.toNSec()/1e3;;
            for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
            {
                pcl_could_3Dbbox_.points.push_back(input_cloud->points[pointIdxRadiusSearch[i]]);
            }
        }
     
        this->pcl_could_3Dbbox = pcl_could_3Dbbox_;
    }   

}

void Mug::ransac_fitting(int obj_idx)
{   

    // random extract points

    // fit the cylinder
        //TODO: now use ros service, switch to a cpp pkg to acclerate

    // geometry_msgs::Point32 point;
    // geometry_msgs::Vector3 axis_direction;
    // Eigen::Quaterniond q;
    // q.x() = state_.position[3];
    // q.y() = state_.position[4];
    // q.z() = state_.position[5];
    // q.w() = state_.position[6];
    // auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    // ROS_INFO_STREAM("Euler from quaternion in roll, pitch, yaw"<< std::endl << euler);
    // axis_direction.x = euler[0];
    // axis_direction.y = euler[0];
    // axis_direction.z = euler[0];

    // int sample_num = 3;
    // for(int i = 0 ; i < sample_num; i ++)
    // {   
    //     point.x = 1.0;
    //     point.y = 1.0;
    //     point.z = 1.0;
    //     cylinder_fit_srv.request.points.push_back(point);
    // }
    // cylinder_fit_srv.request.init_direction = axis_direction;
    
    // if (cylinder_fit_client.call(cylinder_fit_srv))
    // {
    //     ROS_INFO_STREAM("fitting result error :" << cylinder_fit_srv.response.fit_error);
    // }

    if(pcl_could_3Dbbox.points.size() == 0)
    {
        return;
    }
    // RANSAC Infinite Cylinder
    typedef pcl::PointXYZ PointT;
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::PointIndices::Ptr inliers_cylinder_idxPtr(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    
    pcl::copyPointCloud(pcl_could_3Dbbox, *cloud_filtered);
    // *cloud_filtered = pcl_could_3Dbbox;      // take from raw pcl

    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.05);
    seg.setRadiusLimits(0, 0.1);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);
    seg.segment(*inliers_cylinder_idxPtr, *coefficients_cylinder);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inliers_cylinder_Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_filtered, *inliers_cylinder_idxPtr, *inliers_cylinder_Ptr);

    pcl_cloud_vis = *inliers_cylinder_Ptr;   //TODO: use a new pcl object to copy extracted_cloud, better to be a item in a vector
    // std::cerr << "Cylinder coefficients: " << (*coefficients_cylinder) << std::endl;
    // std::cout << "extracted size: " << pcl_cloudXYZ.points.size() << std::endl;


    if(coefficients_cylinder->values.size() > 6)
    {
        // update center axis
        p_center.x = coefficients_cylinder->values[0];
        p_center.y = coefficients_cylinder->values[1];
        p_center.z = coefficients_cylinder->values[2];
        direction_.x = coefficients_cylinder->values[3];
        direction_.y = coefficients_cylinder->values[4];
        direction_.z = coefficients_cylinder->values[5];
    
        // Project inliers onto center axis
        pcl::ModelCoefficients::Ptr coefficients_line (new pcl::ModelCoefficients ());
        coefficients_line->values.resize(6);
        for(int j = 0 ; j < 6; j++)
        {
            coefficients_line->values[j] = coefficients_cylinder->values[j];
        }
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_LINE);
        proj.setInputCloud(inliers_cylinder_Ptr);
        proj.setModelCoefficients(coefficients_line);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
        proj.filter(*cloud_projected);

        pcl_cloud_vis += *cloud_projected;
        pcl_cloud_vis.header.frame_id = pcl_ref_frame;
        pcl_cloud_vis.header.stamp = pcl_time.toNSec()/1e3;;
        
        // get center point and height
           // simply compute the middle point of furthest two points
        pcl::PointXYZ p_1, p_2;
        mug_primitive.body_height = pcl::getMaxSegment(*cloud_projected,p_1,p_2);    
        mug_primitive.body_radius = coefficients_cylinder->values[6];
           // update the pitimitive model
        state_.position[0] = 0.5*(p_1.x-p_2.x) + p_2.x;
        state_.position[1] = 0.5*(p_1.y-p_2.y) + p_2.y;
        state_.position[2] = 0.5*(p_1.z-p_2.z) + p_2.z;

        // TODO: set currently an extra step of primitive pose, only z_axis is measured now
        Eigen::Vector3d p_axis_z, p_axis_x;
        p_axis_z <<  p_1.x-p_2.x, p_1.y-p_2.y, p_1.z-p_2.z;
        p_axis_x << 1,0,0;
        Eigen::Quaterniond q;
        get_orien_from_axisXZ(p_axis_x,p_axis_z,q);
        state_.position[3] = q.x();
        state_.position[4] = q.y();
        state_.position[5] = q.z();
        state_.position[6] = q.w();
    }
    
    return;
}

Mug::Mug(const ros::NodeHandle& nh):nh_(nh),Object(nh)
{
    ROS_INFO("[Object: mug(ros::NodeHandle& nh)]");
    if(sim){}

    // TODO: here is hardcode, move this to yaml params
    this->avg_idx = 0;
    this->handle_idx = 1;
    this->bottom_idx = 2;
    this->top_idx = 3;

    mug_primitive.header.frame_id = ref_frame;
    ROS_INFO_STREAM("primitive state in [ " << ref_frame << " ]");

    // init fitting service
    cylinder_fit_client = nh_.serviceClient<manipulation_msgs::CylinderFit>("/fit_cylinder_with_points");

    // fitting-related pub
    inliers_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/" + object_name + "/inliers_points", 1);
    bboxPCL_pub = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/"+ object_name + "/bbox_points", 1);
    cyliner_line_marker_pub = nh_.advertise<visualization_msgs::Marker>("/"+object_name+"/cylinder_centerline", 1);

    // multi-threading
    {
        std::thread thread_1(&Mug::get3Dbbox_frompointcloud, this, 0);
        thread_1.detach();
    }
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

    if(pcl_cloud_vis.points.size()>0)
    {
        inliers_pub.publish(pcl_cloud_vis);   

        // vis center line
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = pcl_cloudXYZ.header.frame_id;
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.id = 3;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.003;
        line_list.color.b = 0.8;
        line_list.color.a = 0.8;
        geometry_msgs::Point p1, p2;
        float len = 1000;
        p1.x = p_center.x + len*direction_.x;
        p1.y = p_center.y + len*direction_.y;
        p1.z = p_center.z + len*direction_.z;
    
        p2.x = p_center.x - len*direction_.x;
        p2.y = p_center.y - len*direction_.y;
        p2.z = p_center.z - len*direction_.z;

        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
        cyliner_line_marker_pub.publish(line_list);
    }

    if(pcl_could_3Dbbox.points.size()>0)
    {
        bboxPCL_pub.publish(pcl_could_3Dbbox);
    }
}