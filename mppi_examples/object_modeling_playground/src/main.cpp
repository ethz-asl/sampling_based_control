#include "object.cpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_model_node");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(10);

    tf2_ros::TransformBroadcaster broadcaster;

    // mug
    auto mug = Object(nh);
    mug.init_param();
    mug.init_object_publisher("/mug/joint_states", 10);
    mug.trans.header.frame_id = "world";
    mug.trans.child_frame_id = "mug_frame";
        // keypoints gt
    mug.init_kp_array_publisher("/keypoints_marker_array", 10);
    mug.update_kp_markers("mug_frame");
        // fit to cylinder primitive
    mug.fit_primitive();
    mug.init_primitive_array_publisher("/primitives_marker_array",10);
    while (ros::ok())
    {   
        // update mug state and its visulization
        mug.setTF();
        //send the joint state and transform
        //mug.fit_primitive();
        mug.vis_primitive();
        mug.pub_state();
        broadcaster.sendTransform(mug.trans);
        ros::spinOnce();

        loop_rate.sleep();
  }
}