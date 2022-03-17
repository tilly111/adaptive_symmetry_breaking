#!/bin/bash

EXPECTED_ARGS=2
if [ $# -lt ${EXPECTED_ARGS} ]; then
  echo "This script creates N argos files for the cluster!"
  echo "Usage $0 <start> <end> "
  echo $'\t'"[MANDATORY] <start> number of the first experiment"
  echo $'\t'"[MANDATORY] <end> number of the last experiment (first + 4 for a parameter set)"
  exit

else
  cd ARGoS_simulation/
  rm argos3
  cd ..
  ln -s ~/Programs/argos3-kilobot/src/ ARGoS_simulation/argos3
  rm -rf job_cluster/
  nano ARGoS_simulation/behaviours/agent_stub.c
  cd build/
  make
  cd ..
  # here you have to change the specific file you want to run
  sh ARGoS_simulation/data_generation_scripts/cluster_create_argos_files_sample_length.sh ${1} ${2}
  #sh ARGoS_simulation/data_generation_scripts/runjobs.sh
  #watch squeue -u taust
fi