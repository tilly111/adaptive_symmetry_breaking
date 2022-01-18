#!/bin/bash

# make before running experiment
cd ../../build
make
cd ../ARGos_simulation/data_generation_scripts/

###################################
# Synopsis of the script
###################################
EXPECTED_ARGS=2
if [ $# -lt ${EXPECTED_ARGS} ]; then
echo "This script runs N experiments local!"
echo "Usage $0 <start> <end> "
echo $'\t'"[MANDATORY] <start> is the index of the first experiment"
echo $'\t'"[MANDATORY] <end> is the index of the last experiment"
exit

else
    n=3
    NUM_ROBOTS=50
    INITIAL_COMMITMENT=1
    INITIAL_COMMUNICATION_RANGE=2

    EXP_NAME=experiment_adaptation_18_comrng_${INITIAL_COMMUNICATION_RANGE}
    QUORUM=-1 # Quorum to stop experiment

    EXP_LENGTH=2400 #length of the in secs

    DATA_FREQUENCY=1 # frequency of saving the experiment data

    BEHAVIOUR_FILE=/build/behaviours/agent_stub # full path to the compiled robot behaviour
    LOOPFUNCTION_FILE=/build/loop_functions/libkilogrid_stub
	
	#Path variables
  #path to main directory
  EXP_FOLDER=${HOME}/Desktop/ProjectsXCode.nosync/argos3-test

  EXP_TEMPLATE_SRC=../experiment/3_op_template_kilogrid.argos
    
	EXP_DES=${EXP_FOLDER}/experiments_cluster/${n}options_N${NUM_ROBOTS}_${EXP_NAME}
	mkdir -p ${EXP_DES}

	DATA_DES=${EXP_FOLDER}/data_cluster/${n}options_N${NUM_ROBOTS}_${EXP_NAME}
	mkdir -p ${DATA_DES}

	BEHAVIOUR_PATH=${EXP_FOLDER}${BEHAVIOUR_FILE}
  LOOPFUNCTION_PATH=${EXP_FOLDER}${LOOPFUNCTION_FILE}
	
	for i in `seq ${1} ${2}`;
	do

	    EXP_FILE=${EXP_DES}/${EXP_NAME}_${i}.argos # full path to the experiment configuration file

      DATA_FILE=${DATA_DES}/${EXP_NAME}_${i}.txt # Full path to the data file
      CONFIG_FILE=${EXP_FOLDER}/ARGoS_simulation/loop_functions/ASB_experiment_1.kconf
	
	    sed -e "s|exp_length|${EXP_LENGTH}|"                                  \
		      -e "s|randomseed|$(($i*124))|"                                    \
		      -e "s|behaviourpath|${BEHAVIOUR_PATH}|"                           \
          -e "s|loopfunctionpath|${LOOPFUNCTION_PATH}|"                     \
		      -e "s|configfilename|${CONFIG_FILE}|"                             \
		      -e "s|datafilename|${DATA_FILE}|"                                 \
		      -e "s|num_robots|${NUM_ROBOTS}|"                                  \
		      -e "s|initialcommitment|${INITIAL_COMMITMENT}|"                   \
		      -e "s|numberofoptions|${n}|"                                      \
		      -e "s|initialcommunicationrange|${INITIAL_COMMUNICATION_RANGE}|"  \
		        ${EXP_TEMPLATE_SRC} > ${EXP_FILE}

        COMMAND="argos3 -c ${EXP_FILE}"
        ${COMMAND}
        echo experiment ${i} finished...
    done
fi
