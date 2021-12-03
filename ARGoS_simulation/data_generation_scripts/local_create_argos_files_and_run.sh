#!/bin/bash

###################################
# Synopsis of the script
###################################
EXPECTED_ARGS=5
if [ $# -lt ${EXPECTED_ARGS} ]; then
echo "This script generates N experiment files (.argos files)."
echo "Usage $0 <start> <end> <options_size> <model_type> <visualization> <number_of_options>"
echo $'\t'"[MANDATORY] <start> is the index of the first experiment"
echo $'\t'"[MANDATORY] <end> is the index of the last experiment"
echo $'\t'"[MANDATORY] <model_type> is the type of the model (local, global or adaptive)"
echo $'\t'"[MANDATORY] <visualization> visualization flag (0 or 1)"
echo $'\t'"[MANDATORY] <number_of_options> the number of possible options (3 or 5)"
exit

else
    #Varibales to change
    # Experiment name
    rtype=test_adaptive
    EXP_NAME=${rtype}  # Do not forget to change the experiment name.
    
    #TODO: atm hard coded but make it dynamic -> uncommited state to be implemented maybe 0?
    ROBPOP1=50
    ROBPOP2=0
    ROBPOP3=0
    ROBPOP4=0
    ROBPOP5=0
    NUM_ROBOTS=50 # number of robots
    
    #constants
    #Visuaisation flag
    MODEL=${3} # should be local, global or adaptive
    VIZ=${4}
    n=${5}
    GPS_CELLS_NO=20 # GPS resolution: 20 per meter aka every grid cell

    QUORUM=-1 # Quorum to stop experiment  TODO CHANGE BACK
    COMMRANGE=45 # Robots' communication range in kilogrid cells (radius) - init range goes from 0 to 45 this would be global then!
    COMMRANGE_OLD=0.0  # this is for direct robot communication, which we plan to omit!!!
        
    EXP_LENGTH=500 #length of the in secs

    DATA_FREQUENCY=1 # frequency of saving the experiment data

    BEHAVIOUR_FILE=/build/behaviours/agent_${MODEL} # full path to the compiled robot behaviour
    LOOPFUNCTION_FILE=/build/loop_functions/libkilogrid_${MODEL}
	
	#Path variables
    #path to main directory
    EXP_FOLDER=${HOME}/Desktop/ProjectsXCode.nosync/argos3-test
    
    # path to template - either with/out visualization
    if [[ ${VIZ} -eq 1 ]]; then
        EXP_TEMPLATE_SRC=../experiment/${n}_op_template_viz.argos
    else
        EXP_TEMPLATE_SRC=../experiment/${n}_op_template.argos
    fi
    
	EXP_DES=${EXP_FOLDER}/experiments_cluster/${n}options_N${NUM_ROBOTS}_model${MODEL}
	mkdir -p ${EXP_DES}

	DATA_DES=${EXP_FOLDER}/data_cluster/${n}options_N${NUM_ROBOTS}_${MODEL}_${EXP_NAME}
	mkdir -p ${DATA_DES}

	BEHAVIOUR_PATH=${EXP_FOLDER}${BEHAVIOUR_FILE}
    LOOPFUNCTION_PATH=${EXP_FOLDER}${LOOPFUNCTION_FILE}
	
	for i in `seq ${1} ${2}`;
	do

    	NAME_VARIABLE=${MODEL}

	    EXP_FILE=${EXP_DES}/${EXP_NAME}_${NAME_VARIABLE}_${i}.argos # full path to the experiment configuration file

        DATA_FILE=${DATA_DES}/${EXP_NAME}_${n}options_${NAME_VARIABLE}${i}.txt # Full path to the data file
	
	    sed -e "s|exp_length|${EXP_LENGTH}|"           \
		-e "s|randomseed|$(($i*124))|"                 \
		-e "s|behaviourpath|${BEHAVIOUR_PATH}|"        \
        -e "s|loopfunctionpath|${LOOPFUNCTION_PATH}|"  \
		-e "s|data_file|${DATA_FILE}|"                 \
		-e "s|datafrequency|${DATA_FREQUENCY}|"        \
		-e "s|num_robots|${NUM_ROBOTS}|"               \
		-e "s|quorumvalue|${QUORUM}|"                  \
        -e "s|commrng|${COMMRANGE_OLD}|"               \
        -e "s|numberofgpscells|${GPS_CELLS_NO}|"       \
        -e "s|robpop1|${ROBPOP1}|"                     \
        -e "s|robpop2|${ROBPOP2}|"                     \
        -e "s|robpop3|${ROBPOP3}|"                     \
        -e "s|robpop4|${ROBPOP4}|"                     \
        -e "s|robpop5|${ROBPOP5}|"                     \
        -e "s|initcomrng|${COMMRANGE}|"                \
		        ${EXP_TEMPLATE_SRC} > ${EXP_FILE}

        # TODO: when we know ho to schedule on cluster move this outside the loop
        COMMAND="argos3 -c ${EXP_FILE}"
        ${COMMAND}
        echo experiment ${i} finished...
    done
fi
