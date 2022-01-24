#pragma once
#include "object_modeling_playground/object.h"
#include <manipulation_msgs/State.h>
#include <Eigen/Dense>

class Mug: public Object
{
  public:


  
    Mug(const ros::NodeHandle& nh);
    ~Mug() = default;

    void update_TF() override;
    void pub_state() override;
    void update_kp_markers() override;
    void primitive_visualize() override;
    void primitive_estimate() override;
    void update() override;
    void kptoPrimitive();

  private:
    bool estimate_center_pose(Eigen::Vector3d& pos,
                        Eigen::Vector4d& orien);

    double point_to_line(const Eigen::Vector3d& pos_1,
                        const Eigen::Vector3d& pos_2,
                        const Eigen::Vector3d& pos_3);

    Eigen::Matrix3d rot_of_two_frame(const Eigen::Matrix3d& ref_rot,
                            const Eigen::Matrix3d& rel_rot);

    ros::NodeHandle nh_;

    manipulation_msgs::MugPrimitive mug_primitive;
    double center_roll;
    double center_pitch;
    double center_yaw;
    Eigen::Vector3d center_line;
    std::vector<double> height;
    std::vector<double> radius;

    int bottom_idx;
    int top_idx;
    int handle_idx;
    int avg_idx;

};