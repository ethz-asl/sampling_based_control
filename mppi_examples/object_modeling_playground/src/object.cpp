#include "object.h"


void Object::setTF(Eigen::VectorXd state)
{
    state_.header.stamp = ros::Time::now();
    trans.header.stamp = ros::Time::now();
    trans.transform.translation.x = state(0);
    trans.transform.translation.y = state(1);
    trans.transform.translation.z = state(2);
    tf2::Quaternion q_cylinder;
    q_cylinder.setRPY(state(3), state(4), state(5));
    trans.transform.rotation.x = q_cylinder.x();
    trans.transform.rotation.y = q_cylinder.y();
    trans.transform.rotation.z = q_cylinder.z();
    trans.transform.rotation.w = q_cylinder.w();
}

void Object::init_publisher(std::string topic_name, int rate)
{
    state_publisher_=nh_.advertise<sensor_msgs::JointState>(topic_name,rate);
    
}

void Object::pub_state()
{
    state_publisher_.publish(state_);
}