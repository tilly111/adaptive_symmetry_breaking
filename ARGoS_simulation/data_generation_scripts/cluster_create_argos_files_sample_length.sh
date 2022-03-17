#!/bin/bash

###################################
# Synopsis of the script
###################################
EXPECTED_ARGS=2
if [ $# -lt ${EXPECTED_ARGS} ]; then
  echo "This script creates N argos files for the cluster!"
  echo "Usage $0 <start> <end> "
  echo $'\t'"[MANDATORY] <start> number of the first experiment"
  echo $'\t'"[MANDATORY] <end> number of the last experiment ()"
  exit

else
  tmp_counter=0
  MAX_COMMUNICATION_RANGE=30
  INITIAL_COMMUNICATION_RANGE=1
  for j in $(seq ${1} ${2}); do
      # parameters to choose
      INITIAL_COMMITMENT=1 # initial commitment of the robots
      conf=ASB_experiment_4.kconf
      n=2

      # max communication range = sampling number
      if ((${tmp_counter}  == 0)); then
        MAX_COMMUNICATION_RANGE=5
      elif ((${tmp_counter} == 17)); then
        MAX_COMMUNICATION_RANGE=10
      elif ((${tmp_counter} == 34)); then
        MAX_COMMUNICATION_RANGE=15
      elif ((${tmp_counter} == 51)); then
        MAX_COMMUNICATION_RANGE=20
      elif ((${tmp_counter} == 68)); then
        MAX_COMMUNICATION_RANGE=25
      elif ((${tmp_counter} == 85)); then
        MAX_COMMUNICATION_RANGE=30
      elif ((${tmp_counter} == 102)); then
        MAX_COMMUNICATION_RANGE=45
      elif ((${tmp_counter} == 119)); then
        MAX_COMMUNICATION_RANGE=60
      fi
    # initial communication range = communication range
    if (($((${tmp_counter} % 17)) == 0)); then
          INITIAL_COMMUNICATION_RANGE=1
    elif (($((${tmp_counter} % 17)) == 1)); then
          INITIAL_COMMUNICATION_RANGE=2
    elif (($((${tmp_counter} % 17)) == 2)); then
          INITIAL_COMMUNICATION_RANGE=3
    elif (($((${tmp_counter} % 17)) == 3)); then
          INITIAL_COMMUNICATION_RANGE=4
    elif (($((${tmp_counter} % 17)) == 4)); then
          INITIAL_COMMUNICATION_RANGE=5
    elif (($((${tmp_counter} % 17)) == 5)); then
          INITIAL_COMMUNICATION_RANGE=6
    elif (($((${tmp_counter} % 17)) == 6)); then
          INITIAL_COMMUNICATION_RANGE=7
    elif (($((${tmp_counter} % 17)) == 7)); then
          INITIAL_COMMUNICATION_RANGE=8
    elif (($((${tmp_counter} % 17)) == 8)); then
          INITIAL_COMMUNICATION_RANGE=9
    elif (($((${tmp_counter} % 17)) == 9)); then
          INITIAL_COMMUNICATION_RANGE=10
    elif (($((${tmp_counter} % 17)) == 10)); then
          INITIAL_COMMUNICATION_RANGE=15
    elif (($((${tmp_counter} % 17)) == 11)); then
          INITIAL_COMMUNICATION_RANGE=20
    elif (($((${tmp_counter} % 17)) == 12)); then
          INITIAL_COMMUNICATION_RANGE=25
    elif (($((${tmp_counter} % 17)) == 13)); then
          INITIAL_COMMUNICATION_RANGE=30
    elif (($((${tmp_counter} % 17)) == 14)); then
          INITIAL_COMMUNICATION_RANGE=35
    elif (($((${tmp_counter} % 17)) == 15)); then
          INITIAL_COMMUNICATION_RANGE=40
    elif (($((${tmp_counter} % 17)) == 16)); then
          INITIAL_COMMUNICATION_RANGE=45
    fi
    EXP_NAME=experiment_cl_sampling_cross_inhib_02_${j}_comrng_${INITIAL_COMMUNICATION_RANGE}_samples_${MAX_COMMUNICATION_RANGE}
    tmp_counter=$(( ${tmp_counter} + 1 ))

    NUM_ROBOTS=50        # number of robots
    QUORUM=-1            # Quorum to stop experiment NOT USED ATM
    EXP_LENGTH=2400      #length of the experiment in secs
    DATA_FREQUENCY=1     # frequency of saving the experiment data

    HRS=01 # hours the script takes
    MIN=00 # min the script takes

    #path to main directory
    EXP_FOLDER=${HOME}/Programs/adaptive_symmetry_breaking

    # full path to the compiled robot behaviour/loopfunction
    BEHAVIOUR_FILE=${EXP_FOLDER}/build/behaviours/agent_stub
    LOOPFUNCTION_FILE=${EXP_FOLDER}/build/loop_functions/libkilogrid_stub

    CONFIG_FILE=${EXP_FOLDER}/ARGoS_simulation/loop_functions/kilogrid_conf_files/${conf}

    # path to template
    EXP_TEMPLATE_SRC=${EXP_FOLDER}/ARGoS_simulation/experiment/3_op_template_kilogrid.argos
    JOB_TEMPLATE_SRC=${EXP_FOLDER}/ARGoS_simulation/data_generation_scripts/runjob_template.sh

    EXP_DIR=${EXP_FOLDER}/experiments_cluster/${EXP_NAME}
    mkdir -p ${EXP_DIR}

    DATA_DIR=${EXP_FOLDER}/data_cluster/${EXP_NAME}
    mkdir -p ${DATA_DIR}

    JOB_DIR=${EXP_FOLDER}/job_cluster
    mkdir -p ${JOB_DIR}

    for i in $(seq 0 19); do

      EXP_FILE=${EXP_DIR}/${EXP_NAME}_${i}.argos # full path to the experiment configuration file
      JOB_FILE=${JOB_DIR}/${EXP_NAME}_${i}.sh
      DATA_FILE=${EXP_NAME}_${i}.txt # Full path to the data file

      JOB_NAME=${EXP_NAME}_${i}

      sed -e "s|exp_length|${EXP_LENGTH}|" \
        -e "s|randomseed|$(($i * 124))|" \
        -e "s|behaviourpath|${BEHAVIOUR_FILE}|" \
        -e "s|loopfunctionpath|${LOOPFUNCTION_FILE}|" \
        -e "s|configfilename|${CONFIG_FILE}|" \
        -e "s|datafilename|${DATA_FILE}|" \
        -e "s|num_robots|${NUM_ROBOTS}|" \
        -e "s|initialcommitment|${INITIAL_COMMITMENT}|" \
        -e "s|numberofoptions|${n}|" \
        -e "s|initialcommunicationrange|${INITIAL_COMMUNICATION_RANGE}|" \
        -e "s|maxcommunicationrange|${MAX_COMMUNICATION_RANGE}|" \
        -e "s|dynamicenvname|${CONFIG_FILE}|" \
        ${EXP_TEMPLATE_SRC} >${EXP_FILE}

      sed -e "s|jobname|${JOB_NAME}|" \
        -e "s|min|${MIN}|" \
        -e "s|hrs|${HRS}|" \
        -e "s|argosfile|${EXP_FILE}|" \
        -e "s|logfile|${JOB_NAME}|" \
        -e "s|savefile|${DATA_FILE}|" \
        -e "s|wheresave|${DATA_DIR}/|" \
        ${JOB_TEMPLATE_SRC} >${JOB_FILE}
    done
  done
fi
