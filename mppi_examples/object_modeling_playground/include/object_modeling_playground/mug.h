#pragma once
#include "object_modeling_playground/object.h"
#include <manipulation_msgs/State.h>
#include <manipulation_msgs/MugPrimitive.h>
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
  

  private:
    bool estimate_center_pose(Eigen::Vector3d& pos,
                        Eigen::Vector4d& orien);

    double point_to_line(const Eigen::Vector3d& pos_1,
                        const Eigen::Vector3d& pos_2,
                        const Eigen::Vector3d& pos_3);

    double angle_of_two_vector(const Eigen::Vector3d& vec_1,
                            const Eigen::Vector3d& vec_2);

    ros::NodeHandle nh_;

    manipulation_msgs::MugPrimitive mug_primitive;
    double center_roll;
    double center_pitch;
    double center_yaw;
    Eigen::Vector3d center_line;
    std::vector<double> height;
    std::vector<double> radius;

};