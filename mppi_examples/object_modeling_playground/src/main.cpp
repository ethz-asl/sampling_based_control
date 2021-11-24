#include "object.cpp"


void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_model_node");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(10);

    tf2_ros::TransformBroadcaster broadcaster;
    

    // mug
    auto mug = Object(nh);
    //mug.init_param();

        // keypoints gt
    //mug.update_kp_markers("mug_frame");
        // fit to cylinder primitive
    //mug.fit_primitive();


    while (ros::ok())
    {   
        // update mug state and its visulization
        mug.update_obj_TF();
        //send the joint state and transform
        mug.update_kp_markers();

        broadcaster.sendTransform(mug.obj_trans);
        broadcaster.sendTransform(mug.kp_trans);
        
        // primitive fitting
        mug.fit_primitive();
        mug.vis_primitive();

        mug.pub_state();

        ros::spinOnce();

        loop_rate.sleep();
  }
}