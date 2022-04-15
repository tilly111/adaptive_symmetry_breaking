#!/bin/bash

EXPECTED_ARGS=4
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
  # rm -rf job_cluster/
  #nano ARGoS_simulation/behaviours/agent_stub.c
  cd build/
  make
  cd ..
  # here you have to change the specific file you want to run
  # nano ARGoS_simulation/data_generation_scripts/ants_experiments.sh
  sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks.sh 9435 9500 14 204
  sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks.sh 9655 9720 15 204
  sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks.sh 9875 9940 17 204
  sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks.sh 10095 10160 18 204
  echo "change initial commitment to 15"
  nano ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks.sh
  sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks.sh 9501 9566 14 204
  sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks.sh 9721 9786 15 204
  sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks.sh 9941 10006 17 204
  sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks.sh 10161 10226 18 204

  sh ARGoS_simulation/data_generation_scripts/ants_experiments_samples.sh 9567 9654 14 204
  sh ARGoS_simulation/data_generation_scripts/ants_experiments_samples.sh 9787 9874 15 204
  sh ARGoS_simulation/data_generation_scripts/ants_experiments_samples.sh 10007 10094 17 204
  sh ARGoS_simulation/data_generation_scripts/ants_experiments_samples.sh 10227 10314 18 204
  #sh ARGoS_simulation/data_generation_scripts/ants_experiments.sh ${1} ${2} ${3} ${4}
  #sh ARGoS_simulation/data_generation_scripts/runjobs.sh
  #watch squeue -u taust
fi