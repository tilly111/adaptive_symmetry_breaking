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
  MAX_COMMUNICATION_RANGE=30
  INITIAL_COMMUNICATION_RANGE=3
  INITIAL_COMMITMENT=2 # initial commitment of the robots

  # stuff needs to be adjusted
  ENVIRONMENT=${3}
  INITIAL_COMMITMENT_QUALITY=${4}
  for j in $(seq ${1} ${2}); do
      conf=sym_break_${ENVIRONMENT}.kconf
      n=2

      NUM_ROBOTS=$(((25 * (tmp_counter+1))))        # number of robots


      EXP_NAME=experiment_cl_dens_sym_break_cross_inhib_00_${j}_comrng_${INITIAL_COMMUNICATION_RANGE}_nr_${NUM_ROBOTS}_env_${ENVIRONMENT}
      tmp_counter=$(( ${tmp_counter} + 1 ))

      QUORUM=-1            # Quorum to stop experiment NOT USED ATM
      EXP_LENGTH=2400      #length of the experiment in secs
      DATA_FREQUENCY=1     # frequency of saving the experiment data

      HRS=20 # hours the script takes
      MIN=00 # min the script takes

      #path to main directory
      EXP_FOLDER=${HOME}/Programs/adaptive_symmetry_breaking

      # full path to the compiled robot behaviour/loopfunction
      BEHAVIOUR_FILE=${EXP_FOLDER}/build/behaviours/agent_stub
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
      for i in $(seq 0 19); do

          EXP_FILE=${EXP_DIR}/${EXP_NAME}_${i}.argos # full path to the experiment configuration file
          DATA_FILE=${EXP_NAME}_${i}.txt # Full path to the data file

          # it is very important to keep this order in order to not overwrite stuff
          sed -e "s|exp_length|${EXP_LENGTH}|" \
            -e "s|randomseed|$(($i * 124))|" \
            -e "s|behaviourpath|${BEHAVIOUR_FILE}|" \
            -e "s|loopfunctionpath|${LOOPFUNCTION_FILE}|" \
            -e "s|configfilename|${CONFIG_FILE}|" \
            -e "s|datafilename|${DATA_FILE}|" \
            -e "s|num_robots|${NUM_ROBOTS}|" \
            -e "s|initialcommitmentquality|${INITIAL_COMMITMENT_QUALITY}|" \
            -e "s|initialcommitment|${INITIAL_COMMITMENT}|" \
            -e "s|numberofoptions|${n}|" \
            -e "s|initialcommunicationrange|${INITIAL_COMMUNICATION_RANGE}|" \
            -e "s|maxcommunicationrange|${MAX_COMMUNICATION_RANGE}|" \
            -e "s|dynamicenvname|${CONFIG_FILE}|" \
            ${EXP_TEMPLATE_SRC} >${EXP_FILE}


            sed -e "s|argosfile|${EXP_FILE}|" \
              -e "s|wheresave|${DATA_DIR}/|" \
              -e "s|savefile|${DATA_FILE}|" \
            ${JOB_TEMPLATE_SRC_RUN} >>${JOB_FILE}
        done
    done
fi