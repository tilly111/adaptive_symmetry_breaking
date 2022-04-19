#!/bin/bash

###################################
# script for ants paper - sample time vs problem difficulty
###################################
EXPECTED_ARGS=4
if [ $# -lt ${EXPECTED_ARGS} ]; then
  echo "This script creates N argos files for the cluster!"
  echo "Usage $0 <start> <end> "
  echo $'\t'"[MANDATORY] <start> number of the first experiment"
  echo $'\t'"[MANDATORY] <end> number of the last experiment ()"
  echo $'\t'"[MANDATORY] <environment> number of the environment"
  echo $'\t'"[MANDATORY] <init quality> init quality"
  exit

else
  tmp_counter=0
  MAX_COMMUNICATION_RANGE=2
  INITIAL_COMMUNICATION_RANGE=1
  INITIAL_COMMITMENT=10 # currently Sample counter max

  # stuff needs to be adjusted
  ENVIRONMENT=${3}
  INITIAL_COMMITMENT_QUALITY=${4}
  for j in $(seq ${1} ${2}); do
      # parameters to choose

      # TODO adjust world name + and number of options
      conf=sample_${ENVIRONMENT}
      n=2

      # max communication range = sampling number
      if ((${tmp_counter}  == 0)); then
        INITIAL_COMMITMENT=5
      elif ((${tmp_counter} == 11)); then
        INITIAL_COMMITMENT=10
      elif ((${tmp_counter} == 22)); then
        INITIAL_COMMITMENT=15
      elif ((${tmp_counter} == 33)); then
        INITIAL_COMMITMENT=20
      elif ((${tmp_counter} == 44)); then
        INITIAL_COMMITMENT=25
      elif ((${tmp_counter} == 55)); then
        INITIAL_COMMITMENT=30
      elif ((${tmp_counter} == 66)); then
        INITIAL_COMMITMENT=45
      elif ((${tmp_counter} == 77)); then
        INITIAL_COMMITMENT=60
      fi
    # initial communication range = communication range
    if (($((${tmp_counter} % 11)) == 0)); then  # change back to 17 ..
          INITIAL_COMMUNICATION_RANGE=1
    elif (($((${tmp_counter} % 11)) == 1)); then
          INITIAL_COMMUNICATION_RANGE=2
    elif (($((${tmp_counter} % 11)) == 2)); then
          INITIAL_COMMUNICATION_RANGE=3
    elif (($((${tmp_counter} % 11)) == 3)); then
          INITIAL_COMMUNICATION_RANGE=4
    elif (($((${tmp_counter} % 11)) == 4)); then
          INITIAL_COMMUNICATION_RANGE=5
    elif (($((${tmp_counter} % 11)) == 5)); then
          INITIAL_COMMUNICATION_RANGE=6
    elif (($((${tmp_counter} % 11)) == 6)); then
          INITIAL_COMMUNICATION_RANGE=7
    elif (($((${tmp_counter} % 11)) == 7)); then
          INITIAL_COMMUNICATION_RANGE=8
    elif (($((${tmp_counter} % 11)) == 8)); then
          INITIAL_COMMUNICATION_RANGE=9
#    elif (($((${tmp_counter} % 17)) == 9)); then
#          INITIAL_COMMUNICATION_RANGE=10
#    elif (($((${tmp_counter} % 17)) == 10)); then
#          INITIAL_COMMUNICATION_RANGE=15
#    elif (($((${tmp_counter} % 17)) == 11)); then
#          INITIAL_COMMUNICATION_RANGE=20
    elif (($((${tmp_counter} % 11)) == 9)); then
          INITIAL_COMMUNICATION_RANGE=25
#    elif (($((${tmp_counter} % 17)) == 13)); then
#          INITIAL_COMMUNICATION_RANGE=30
#    elif (($((${tmp_counter} % 17)) == 14)); then
#          INITIAL_COMMUNICATION_RANGE=35
#    elif (($((${tmp_counter} % 17)) == 15)); then
#          INITIAL_COMMUNICATION_RANGE=40
    elif (($((${tmp_counter} % 11)) == 10)); then
          INITIAL_COMMUNICATION_RANGE=45
    fi
    EXP_NAME=cl_sample_cross_inhib_00_${j}_comrng_${INITIAL_COMMUNICATION_RANGE}_samples_${INITIAL_COMMITMENT}_env_${ENVIRONMENT}
    tmp_counter=$(( ${tmp_counter} + 1 ))

    NUM_ROBOTS=50        # number of robots
    QUORUM=-1            # Quorum to stop experiment NOT USED ATM
    EXP_LENGTH=2400      #length of the experiment in secs
    DATA_FREQUENCY=1     # frequency of saving the experiment data

    HRS=23 # hours the script takes
    MIN=59 # min the script takes

    #path to main directory
    EXP_FOLDER=${HOME}/Programs/adaptive_symmetry_breaking

    # full path to the compiled robot behaviour/loopfunction
    # TODO change back controller
    BEHAVIOUR_FILE=${EXP_FOLDER}/build/behaviours/agent_perfect_sample
    LOOPFUNCTION_FILE=${EXP_FOLDER}/build/loop_functions/libkilogrid_stub

    CONFIG_FILE=${EXP_FOLDER}/ARGoS_simulation/loop_functions/kilogrid_conf_files/${conf}

    # path to template
    EXP_TEMPLATE_SRC=${EXP_FOLDER}/ARGoS_simulation/experiment/3_op_template_kilogrid.argos
    JOB_TEMPLATE_SRC=${EXP_FOLDER}/ARGoS_simulation/data_generation_scripts/runjob_template_master.sh
    JOB_TEMPLATE_SRC_RUN=${EXP_FOLDER}/ARGoS_simulation/data_generation_scripts/runjob_template_run.sh

    EXP_DIR=${EXP_FOLDER}/experiments_cluster/${EXP_NAME}
    mkdir -p ${EXP_DIR}

    DATA_DIR=${EXP_FOLDER}/data_cluster/${EXP_NAME}
    mkdir -p ${DATA_DIR}

    JOB_DIR=${EXP_FOLDER}/job_cluster
    mkdir -p ${JOB_DIR}

    JOB_FILE=${JOB_DIR}/${EXP_NAME}.sh
    JOB_NAME=${EXP_NAME}
    # creating header for new script
    sed -e "s|jobname|${JOB_NAME}|" \
      -e "s|min|${MIN}|" \
      -e "s|hrs|${HRS}|" \
      -e "s|logfile|${JOB_NAME}|" \
      ${JOB_TEMPLATE_SRC} >${JOB_FILE}
    # TODO change back to higher number
    for i in $(seq 0 19); do

      EXP_FILE=${EXP_DIR}/${EXP_NAME}_${i}.argos # full path to the experiment configuration file
      DATA_FILE=${EXP_NAME}_${i}.txt # Full path to the data file
      CONFIG_FILE_FINAL=${CONFIG_FILE}_${i}.kconf

      # it is very important to keep this order in order to not overwrite stuff
      sed -e "s|exp_length|${EXP_LENGTH}|" \
        -e "s|randomseed|$(($i * 124))|" \
        -e "s|behaviourpath|${BEHAVIOUR_FILE}|" \
        -e "s|loopfunctionpath|${LOOPFUNCTION_FILE}|" \
        -e "s|configfilename|${CONFIG_FILE_FINAL}|" \
        -e "s|datafilename|${DATA_FILE}|" \
        -e "s|num_robots|${NUM_ROBOTS}|" \
        -e "s|initialcommitmentquality|${INITIAL_COMMITMENT_QUALITY}|" \
        -e "s|initialcommitment|${INITIAL_COMMITMENT}|" \
        -e "s|numberofoptions|${n}|" \
        -e "s|initialcommunicationrange|${INITIAL_COMMUNICATION_RANGE}|" \
        -e "s|maxcommunicationrange|${MAX_COMMUNICATION_RANGE}|" \
        -e "s|dynamicenvname|${CONFIG_FILE_FINAL}|" \
        ${EXP_TEMPLATE_SRC} >${EXP_FILE}


        sed -e "s|argosfile|${EXP_FILE}|" \
          -e "s|wheresave|${DATA_DIR}/|" \
          -e "s|savefile|${DATA_FILE}|" \
        ${JOB_TEMPLATE_SRC_RUN} >>${JOB_FILE}
    done
  done
fi
