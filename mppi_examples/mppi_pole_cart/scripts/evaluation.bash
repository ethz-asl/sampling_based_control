#!/usr/bin/env bash
n_runs=5
while getopts n:p: opts; do
   case ${opts} in
      n) n_runs=${OPTARG} ;;
      p) policy_path=${OPTARG} ;;
   esac
done

data_recorder_path=$(rospack find mppi_ros)/scripts/record_data.py

for ((i=0; i<$n_runs; i++))
do
    echo "*****Start********"
    python3 $data_recorder_path&

    roslaunch mppi_pole_cart pole_cart_evaluation.launch \
    torchscript_model_path:=$policy_path 
    echo "Stoppping data recorder"
    kill $!
    sleep 1
    echo "*****DONE********"
done



