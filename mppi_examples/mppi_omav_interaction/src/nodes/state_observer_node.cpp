//
// Created by studigem on 13.06.21.
//
#include <ros/ros.h>
#include "mppi_omav_interaction/state_observer.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_observer_object");
    ros::NodeHandle nh("~");

    double frequency;
    frequency = 250;

    object_observer::StateObserver observer(nh);
    if (!observer.initialize()) {
        ROS_ERROR("Failed to initialize the state observer.");
        return 0;
    }
    bool center_calculated;
    while (!center_calculated) {
      center_calculated = observer.estimateCenter();
    }

    ros::Rate rate(frequency);
    while (ros::ok()) {
        observer.publish();

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
