#!/usr/bin/env bash

while getopts c:n: opts; do
   case ${opts} in
      c) CVS_PATH=${OPTARG} ;;
      n) USER_NAME=${OPTARG} ;;
   esac
done

# dir is relative and should work on any machine as long as file structure of
# package remains the same
script_dir="$(dirname "$0")"
data_dir="$script_dir/../../../policy_learning/data/"
cd $data_dir
dataset_name=$USER_NAME"_"$(date +%m%d%H%M%S)
mkdir $dataset_name
cd $dataset_name
current_dir=$(pwd)

if [ -z "$CVS_PATH" ]
  then
    echo "No data csv provided. Generate own random data."

      # loop over random inital conditions and save them to csv
    echo "x_position,x_dot,theta,theta_dot" >> value_log.csv
    n_runs=50
    for ((i=0; i<$n_runs; i++))
    do
      # The random variable changed with the modulo does not actually yield proper
      # uniform distribution, only an approximation where the tail end of the numbers
      # are less likely (i.e. higher numbers are marginally less likely then smaller ones)
      x_position=$(echo "scale=2; (-1^$(($RANDOM%2)))*($(($RANDOM%20))/10)" |bc)
      x_dot=$(echo "scale=2; (-1^$(($RANDOM%2)))*($(($RANDOM%5))/10)" |bc)
      PI=3.14
      theta=$(echo "scale=2; (-1^$(($RANDOM%2)))*($(($RANDOM%300))/100)" |bc)
      theta_dot=$(echo "scale=2; (-1^$(($RANDOM%2)))*($(($RANDOM%5))/10)" |bc)

      # log the random values for reproducibility
      echo $x_position","$x_dot","$theta","$theta_dot >> value_log.csv
    done
    echo "Data written to "$current_dir"/value_log.csv"
  else
    cp $CVS_PATH value_log.csv
fi

i=0
while IFS="," read -r x_position x_dot theta theta_dot
do
  output_file="/run_${i}.hdf5"
  output_path=$current_dir$output_file
  # starts the simulation with the data collection option and the specified
  # initial condition
  roslaunch mppi_pole_cart pole_cart_data_collection.launch \
  learner_output_path:=$output_path \
  x:=${x_position} x_d:=${x_dot} \
  theta:=${theta} theta_d:=${theta_dot}

  let "i+=1" 
done < <(tail -n +2 value_log.csv)



