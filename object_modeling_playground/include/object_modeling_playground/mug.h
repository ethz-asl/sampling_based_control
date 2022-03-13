#pragma once
#include "object_modeling_playground/object.h"
#include <manipulation_msgs/State.h>
#include <Eigen/Dense>
//#include "object_modeling_playground/cylinder_fit.h"
#include <manipulation_msgs/CylinderFit.h>

// PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl_ros/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>

class Mug: public Object
{
  public:
  
    Mug(const ros::NodeHandle& nh);
    ~Mug() = default;

    void update_TF() override;
    void pub_state() override;
    void primitive_visualize() override;
    bool primitive_estimate(int obj_idx) override;
    void update() override;
    void kptoPrimitive();

  private:
    // Esimate one object's center 6D Pose
    bool estimate_center_pose(Eigen::Vector3d& pos,
                        Eigen::Vector4d& orien,
                        int obj_idx);

    // Generate Quaternion from two direction axis: X and Z 
      // axis_x z: Two vector of directions(doesn't have to be orthogonal)
      // quat: Resulting quaternion estimated from the given axis
    void get_orien_from_axisXZ(Eigen::Vector3d& axis_x,
                        Eigen::Vector3d& axis_z,
                        Eigen::Quaterniond& quat);

    double point_to_line(const Eigen::Vector3d& pos_1,
                        const Eigen::Vector3d& pos_2,
                        const Eigen::Vector3d& pos_3);

    void ransac_fitting(int obj_idx);
    void get3Dbbox_frompointcloud(int obj_idx);

    Eigen::Matrix3d rot_of_two_frame(const Eigen::Matrix3d& ref_rot, const Eigen::Matrix3d& rel_rot);
    Eigen::Vector3d get_pt_from_kpArray(int obj_idx, int pt_idx);

    // fitting 
    manipulation_msgs::MugPrimitive mug_primitive;
    Eigen::Vector3d center_line;
    std::vector<double> height;
    std::vector<double> radius;

    int bottom_idx;
    int top_idx;
    int handle_idx;
    int avg_idx;

    // service
    ros::ServiceClient cylinder_fit_client;
    manipulation_msgs::CylinderFit cylinder_fit_srv; 
    
    // ros 
    ros::NodeHandle nh_;
    ros::Publisher inliers_pub, bboxPCL_pub;
    ros::Publisher cyliner_line_marker_pub;
    geometry_msgs::Point p_center;
    geometry_msgs::Vector3 direction_;

    // PCL
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_vis, pcl_could_3Dbbox;  // pointcloud for visulization
};