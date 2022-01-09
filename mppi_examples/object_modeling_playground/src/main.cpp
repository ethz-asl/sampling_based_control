#include "object_modeling_playground/mug.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_model_node");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(10);

    // init one specific objects 
    auto mug = std::make_unique<Mug>(nh);
    while (ros::ok())
    {
        mug->update();
        ros::spinOnce();
        loop_rate.sleep();
    }
}