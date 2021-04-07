#!/usr/bin/env bash

dir is relative and should work on any machine as long as file structure of
package remains the same
data_dir="../../../policy_learning/data/"
cd $data_dir
current_time=$(date +%m%d%H%M%S)
mkdir $current_time
cd $current_time
current_dir=$(pwd)
# loop over random inital conditions, for each initial condtition save a new
# data file
for ((i=0; i<10; i++))
do
  output_file="/run_${i}.hdf5"
  output_path=$current_dir$output_file
  # The random variable changed with the modulo does not actually yield proper
  # uniform distribution, only an approximation where the tail end of the numbers
  # are less likely (i.e. higher numbers are marginally less likely then smaller ones)
  x_position=$(echo "scale=2; (-1^$(($RANDOM%2)))*($(($RANDOM%20))/10)" |bc)
  x_dot=$(echo "scale=2; (-1^$(($RANDOM%2)))*($(($RANDOM%5))/10)" |bc)
  PI=3.14
  theta=$(echo "scale=2; (-1^$(($RANDOM%2)))*($(($RANDOM%300))/100)" |bc)
  theta_dot=$(echo "scale=2; (-1^$(($RANDOM%2)))*($(($RANDOM%5))/10)" |bc)
  # echo "x: "$x_position
  # echo "x_d: "$x_dot
  # echo "theta: "$theta
  # echo "theta_d: "$theta_dot
  roslaunch mppi_pole_cart pole_cart_data_collection.launch \
  learner_output_path:=$output_path \
  x:=${x_position} x_d:=${x_dot} \
  theta:=${theta} theta_d:=${theta_dot}
done
