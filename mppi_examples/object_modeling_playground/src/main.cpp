#include "object.cpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_model_node");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(10);

    tf2_ros::TransformBroadcaster broadcaster;

    // mug
    auto mug = Object(nh);
    mug.init_publisher("/mug/joint_states", 10);
    mug.trans.header.frame_id = "world";
    mug.trans.child_frame_id = "mug_frame";
    
    // keypoints gt
    while (ros::ok())
    {   
        // update mug state and its visulization
        Eigen::VectorXd mug_state(6);
        mug_state << 0,0,1,0,0,0;
        mug.setTF(mug_state);

        //send the joint state and transform
        mug.pub_state();
        broadcaster.sendTransform(mug.trans);

        ros::spinOnce();

        loop_rate.sleep();
  }
}