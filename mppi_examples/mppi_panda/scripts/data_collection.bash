#!/usr/bin/env bash

# dir is relative and should work on any machine as long as file structure of
# package remains the same
data_dir="../../../policy_learning/data/"
cd $data_dir
current_time=$(date +%m%d%H%M%S)
mkdir $current_time
cd $current_time
current_dir=$(pwd)
# loop over random inital conditions, for each initial condtition save a new
# data file
echo "joint_1,joint_2,joint_3,joint_4,joint_5,joint_6,joint_7" >> value_log.csv
n_runs=1
for ((i=0; i<$n_runs; i++))
do
  output_file="/run_${i}.hdf5"
  output_path=$current_dir$output_file

  # The random variable changed with the modulo does not actually yield proper
  # uniform distribution, only an approximation where the tail end of the numbers
  # are less likely (i.e. higher numbers are marginally less likely then smaller ones)
  # joint limits:
  # Lower joints limits: -2.8973 -1.7628 -2.8973 -3.0718 -2.8973 -0.0175 -2.8973
  # Upper joints limits: 2.8973 1.7628 2.8973 0.0698 2.8973 3.7525 2.8973
  joint_1=$(echo "scale=2; (-1^$(($RANDOM%2)))*($(($RANDOM%29))/10)" |bc \
    | awk '{ printf("%1.2f\n",$1) }')
  joint_2=$(echo "scale=2; (-1^$(($RANDOM%2)))*($(($RANDOM%18))/10)" |bc \
    | awk '{ printf("%1.2f\n",$1) }')
  joint_3=$(echo "scale=2; (-1^$(($RANDOM%2)))*($(($RANDOM%30))/10)" |bc \
    | awk '{ printf("%1.2f\n",$1) }')
  joint_4=$(echo "scale=2; (-1^$(($RANDOM%2)))*($(($RANDOM%31))/10)" |bc \
    | awk '{ printf("%1.2f\n",$1) }')
  joint_5=$(echo "scale=2; (-1^$(($RANDOM%2)))*($(($RANDOM%29))/10)" |bc \
    | awk '{ printf("%1.2f\n",$1) }')
  joint_7=$(echo "scale=2; (-1^$(($RANDOM%2)))*($(($RANDOM%29))/10)" |bc \
    | awk '{ printf("%1.2f\n",$1) }')
  # asymmetric joint limit
  joint_6=$(echo "scale=2; ($(($RANDOM%29))/10)" |bc \
    | awk '{ printf("%1.2f\n",$1) }')

  # random goal positions - they don't need to be in the possible space
  # orientation (normalization is handled by the callback)
  q_x=$(echo "scale=2; (-1^$(($RANDOM%2)))*($(($RANDOM%100))/100)" |bc \
    | awk '{ printf("%1.2f\n",$1) }')
  q_y=$(echo "scale=2; (-1^$(($RANDOM%2)))*($(($RANDOM%100))/100)" |bc \
    | awk '{ printf("%1.2f\n",$1) }')
  q_z=$(echo "scale=2; (-1^$(($RANDOM%2)))*($(($RANDOM%100))/100)" |bc \
    | awk '{ printf("%1.2f\n",$1) }')
  q_w=$(echo "scale=2; (-1^$(($RANDOM%2)))*($(($RANDOM%100))/100)" |bc \
    | awk '{ printf("%1.2f\n",$1) }')
  # position
  x_pos=$(echo "scale=2; (-1^$(($RANDOM%2)))*($(($RANDOM%15))/10)" |bc \
    | awk '{ printf("%1.2f\n",$1) }')
  y_pos=$(echo "scale=2; (-1^$(($RANDOM%2)))*($(($RANDOM%15))/10)" |bc \
    | awk '{ printf("%1.2f\n",$1) }')
  z_pos=$(echo "scale=2; (-1^$(($RANDOM%2)))*($(($RANDOM%15))/10)" |bc \
    | awk '{ printf("%1.2f\n",$1) }')


  # log the random values for reproducibility
  echo $joint_1","$joint_2","$joint_3","$joint_3","$joint_4","$joint_5","\
    $joint_6","$joint_7 >> value_log.csv
  initial_cond_vec=`echo '['$joint_1', '$joint_2', '$joint_3', '$joint_4', '\
    $joint_5', '$joint_6', '$joint_7']'`
  echo "$initial_cond_vec"

  goal_pos_cmd=`echo '{pose: {position: {x: '$x_pos', y: '$y_pos', z: '$z_pos'},'\
    'orientation: {x: '$q_x', y: '$q_y', z: '$q_z', w: '$q_w'}}}'`
  echo "$goal_pos_cmd"

  # starts the simulation with the data collection option and the specified
  # initial condition & sets the desired endeffector pose in cartesian space
  roslaunch mppi_panda panda_data_collection.launch \
  learner_output_path:=$output_path \
  initial_condition_vector:="$initial_cond_vec" &>/dev/null \
  & sleep 10s; rostopic pub /end_effector_pose_desired geometry_msgs/PoseStamped \
  "$goal_pos_cmd"

done
