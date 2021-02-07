#!/bin/bash

SAMPLES_EXPERIMENT='20 60 80 100'

for SAMPLES in $SAMPLES_EXPERIMENT
do
  yq write --inplace -- $PWD/../config/params.yaml 'solver.rollouts' $SAMPLES
  roslaunch mppi_panda panda_control.launch &>/dev/null
  PROCESS_ID=$!
  sleep 2s
  rosrun mppi_panda target_generator.py
  sleep 22s
  kill -INT $PROCESS_ID
done

#roslaunch mppi_panda panda_control.launch &
#PROCESS_ID=$!

#echo "New experiment launched with pid: ${PROCESS_ID}"
#sleep 20s
#kill -INT $PROCESS_ID

# Requires yq
#sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys CC86BB64
#sudo add-apt-repository ppa:rmescandon/yq
#sudo apt update
#sudo apt install yq -y