#!/usr/bin/env bash

output_path="/home/kiran/test_file_new.hdf5"
x_position=2.0
x_dot=0
PI=3.14
theta=${PI}
theta_dot=0
roslaunch mppi_pole_cart pole_cart_data_collection.launch \
learner_output_path:=$output_path \
x:=${x_position} x_d:=${x_dot} \
theta:=${theta} theta_d:=${theta_dot}
