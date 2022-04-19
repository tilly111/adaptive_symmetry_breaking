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


  # symmetry breaking
  # check for setted to distributed initial commitment
  nano ARGoS_simulation/behaviours/agent_stub.c
  sh ARGoS_simulation/data_generation_scripts/ants_symmetry_breaking.sh 11155 11242 2 2
  sh ARGoS_simulation/data_generation_scripts/ants_symmetry_breaking.sh 11243 11330 3 3
  sh ARGoS_simulation/data_generation_scripts/ants_symmetry_breaking.sh 11331 11418 4 4
  sh ARGoS_simulation/data_generation_scripts/ants_symmetry_breaking.sh 11419 11506 5 5

  # perfect distribution
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_samples.sh 10935 11022 17 204
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks.sh 11023 11088 17 204 10
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks.sh 11089 11154 17 204 15



  # density experiment
#   sh ARGoS_simulation/data_generation_scripts/ants_dens_adapt.sh 10315 10394 14 204
#   sh ARGoS_simulation/data_generation_scripts/ants_dens_adapt.sh 10395 10474 15 204
#   sh ARGoS_simulation/data_generation_scripts/ants_dens_adapt.sh 10475 10554 16 204
#   sh ARGoS_simulation/data_generation_scripts/ants_dens_adapt.sh 10555 10634 17 204
#   sh ARGoS_simulation/data_generation_scripts/ants_dens_adapt.sh 10635 10714 18 204


  #sh ARGoS_simulation/data_generation_scripts/ants_experiments.sh ${1} ${2} ${3} ${4}
  #sh ARGoS_simulation/data_generation_scripts/runjobs.sh
  #watch squeue -u taust
fi