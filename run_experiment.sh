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
#   sh ARGoS_simulation/data_generation_scripts/ants_symmetry_breaking.sh 11155 11242 2 2
#   sh ARGoS_simulation/data_generation_scripts/ants_symmetry_breaking.sh 11243 11330 3 3
#   sh ARGoS_simulation/data_generation_scripts/ants_symmetry_breaking.sh 11331 11418 4 4
#   sh ARGoS_simulation/data_generation_scripts/ants_symmetry_breaking.sh 11419 11506 5 5

  # symmetry breaking - density
#   sh ARGoS_simulation/data_generation_scripts/ants_dens_sym_break.sh 11507 11586 2 2
#   sh ARGoS_simulation/data_generation_scripts/ants_dens_sym_break.sh 11587 11666 3 3
#   sh ARGoS_simulation/data_generation_scripts/ants_dens_sym_break.sh 11667 11746 4 4
#   sh ARGoS_simulation/data_generation_scripts/ants_dens_sym_break.sh 11747 11826 5 5

  # perfect distribution
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_samples.sh 10935 11022 17 204
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks.sh 11023 11088 17 204 10
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks.sh 11089 11154 17 204 15
  # no noise
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_samples.sh 11827 11914 17 204
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks.sh 11915 11980 17 204 5
  # noise sigma = 0.1, last value is sigma
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_samples_ps.sh 11981 12068 15 204 2
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks_ps.sh 12069 12134 15 204 5 2
  # noise sigma = 0.2, last value is sigma
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_samples_ps.sh 12135 12222 15 204 4
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks_ps.sh 12223 12288 15 204 5 4
  # extension 6 and 7 sec
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks_ps.sh 12223 12288 15 204 5 4
  # noise sigma = 0.3, last value is sigma
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_samples_ps.sh 12289 12376 15 204 6
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks_ps.sh 12377 12442 15 204 5 6
  # extension 6 and 7 sec
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks_ps.sh 13009 13030 15 204 5 6

  # normal with 15 ticks and denser results map
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_samples_ps.sh 12443 12578 15 204  # need 136 slots
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks.sh 12579 12680 15 204 15  # need to be 102
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks.sh 12681 12782 13 204 15  # need to be 102
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks.sh 12783 12884 17 204 15  # need to be 102
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks.sh 12885 12986 18 204 15  # need to be 102

  # density experiment
#   sh ARGoS_simulation/data_generation_scripts/ants_dens_adapt.sh 10315 10394 14 204
#   sh ARGoS_simulation/data_generation_scripts/ants_dens_adapt.sh 10395 10474 15 204
#   sh ARGoS_simulation/data_generation_scripts/ants_dens_adapt.sh 10475 10554 16 204
#   sh ARGoS_simulation/data_generation_scripts/ants_dens_adapt.sh 10555 10634 17 204
#   sh ARGoS_simulation/data_generation_scripts/ants_dens_adapt.sh 10635 10714 18 204

  # SYMMETRY BREAKING FINAL EXPERIMENTS
#   sh ARGoS_simulation/data_generation_scripts/ants_symmetry_breaking.sh 13031 13166 2 2
#   sh ARGoS_simulation/data_generation_scripts/ants_symmetry_breaking.sh 13167 13302 3 3
#   sh ARGoS_simulation/data_generation_scripts/ants_symmetry_breaking.sh 13303 13438 4 4
#   sh ARGoS_simulation/data_generation_scripts/ants_symmetry_breaking.sh 13439 13574 5 5

  # ANTS FINAL EXPERIMENTS:
  # TODO (b) normal with kappa 0.9 ATTENTION set initial commitment in controller to 1
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_samples.sh 13711 13846 17 204

  # TODO (c) adaptation with 15 samples kappa 0.9 vary time between samples
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks.sh 13847 13982 17 204 15

  # (g) adaptation with 5 samples different time between samples 2 options kappa=0.9 sigma=0.1
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks_ps.sh 13983 14118 17 204 5 2

  # (h) adaptation with 5 samples different time between samples 2 options kappa=0.9 sigma=0.2
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks_ps.sh 13575 13710 17 204 5 4

  # g replacement! -> adaptation with 5 samples different time between samples 2 options kappa=0.9 sigma=0.0
#   sh ARGoS_simulation/data_generation_scripts/ants_experiments_sample_ticks_ps.sh 14119 14254 17 204 5 0

  # adaptation for different kappa .. master thesis and paper
  sh ARGoS_simulation/data_generation_scripts/ants_experiments_samples.sh 14255 14390 13 204
  sh ARGoS_simulation/data_generation_scripts/ants_experiments_samples.sh 14391 14526 15 204
  sh ARGoS_simulation/data_generation_scripts/ants_experiments_samples.sh 13711 13846 17 204
  sh ARGoS_simulation/data_generation_scripts/ants_experiments_samples.sh 14527 14662 18 204



  #sh ARGoS_simulation/data_generation_scripts/ants_experiments.sh ${1} ${2} ${3} ${4}
  #sh ARGoS_simulation/data_generation_scripts/runjobs.sh
  #watch squeue -u taust
fi