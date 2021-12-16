#include "object_modeling_playground/mug.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_model_node");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(10);

    tf2_ros::TransformBroadcaster broadcaster;
    
    // init one specific objects 
    auto mug = std::make_unique<Mug>(nh);
    ros::spin();
}