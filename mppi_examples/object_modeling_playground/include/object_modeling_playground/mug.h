#pragma once
#include "object_modeling_playground/object.h"

class Mug: public Object
{
  public:


  
    Mug(const ros::NodeHandle& nh);
    ~Mug() = default;

    void update_obj_TF() override;
    void pub_state() override;
    void update_kp_markers() override;
    void fit_primitive() override;
    void vis_primitive() override;
    void pose_estimate() override;


  private:
    ros::NodeHandle nh_;
    std::vector<int> set_nums;
};