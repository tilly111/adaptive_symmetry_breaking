#!/bin/bash

###################################
# Synopsis of the script
###################################
EXPECTED_ARGS=6
if [ $# -lt ${EXPECTED_ARGS} ]; then
echo "This script generates N experiment files (.argos files)."
echo "Usage $0 <start> <end> <options_size> <model_type> <visualization> <number_of_options> <on_cluster>"
echo $'\t'"[MENDATORY] <start> is the index of the first experiment"
echo $'\t'"[MENDATORY] <end> is the index of the last experiment"
echo $'\t'"[MENDATORY] <model_type> is the type of the model (local, global or adaptive)"
echo $'\t'"[MENDATORY] <visualization> visualization flag (0 or 1)"
echo $'\t'"[MENDATORY] <number_of_options> the number of possible options (3 or 5)"
echo $'\t'"[MENDATORY] <on_cluster> flag if run on cluster (0 or 1)"
exit

else
    
    HPC="i"  #i-->iceberg s-->sharc ???

    #Varibales to change
    # Experiment name
    rtype=test
    EXP_NAME=${rtype}  # Do not forget to change the experiment name.
    
    #TODO: atm hard coded but make it dynamic -> uncommited state to be implemented maybe 0?
    ROBPOP1=50
    ROBPOP2=0
    ROBPOP3=0
    ROBPOP4=0
    ROBPOP5=0
    
    #constants
    #Visuaisation flag
    MODEL=${3} # should be "compare" or "inhib"
    VIZ=${4}
    n=${5}
    CLUSTER=${6}
    NUM_ROBOTS=50 # number of robots
    GPS_CELLS_NO=20 # GPS resolution: 20 per meter aka every grid cell

    QUORUM=0.9 # Quorum to stop experiment  TODO CHANGE BACK
    COMMRANGE=0.1 # Robots' communication range - not needed ?
        
    EXP_LENGTH=2400 #length of the in secs

    DATA_FREQUENCY=1 # frequency of saving the experiment data

    BEHAVIOUR_FILE=/build/behaviours/agent_${MODEL} # full path to the compiled robot behaviour
    LOOPFUNCTION_FILE=/build/loop_functions/libkilogrid_${MODEL}

    # TODO: what does this one do?
	if [[ ${NUM_ROBOTS} -eq 50 ]]; then
		HRS="01"
		MIN="00"
	else
		HRS="02"
		MIN="00"
	fi

	#Set job name to distanguish between jobs on the queu
	JOB_NAME=${rtype}_${NUM_ROBOTS}_model${MODEL}
	sed -e "s|start|${1}|"            \
	    -e "s|end|${2}|"              \
	    -e "s|jobname|${JOB_NAME}|"   \
	    -e "s|min|${MIN}|"            \
	    -e "s|hrs|${HRS}|"            \
	    runjob_template.sh > runjob.sh
	
	#Path variables
    #path to main directory
    if [[ ${CLUSTER} -eq 1 ]]; then
        EXP_FOLDER=${HOME}/Programs/argos3-test
    else
        EXP_FOLDER=${HOME}/Desktop/ProjectsXCode.nosync/argos3-test
    fi
	
    # path to template - either with/out visualization
    if [[ ${VIZ} -eq 1 ]]; then
        EXP_TEMPLATE_SRC=ARGoS_simulation/experiment/${n}_op_template_viz.argos
    else
        EXP_TEMPLATE_SRC=ARGoS_simulation/experiment/${n}_op_template.argos
    fi
    
	EXP_DES=${EXP_FOLDER}/experiments_cluster/${n}options_N${NUM_ROBOTS}_model${MODEL}
	mkdir -p ${EXP_DES}

	DATA_DES=${EXP_FOLDER}/data_cluster/${n}options_N${NUM_ROBOTS}_${MODEL}_${EXP_NAME}
	mkdir -p ${DATA_DES}

	BEHAVIOUR_PATH=${EXP_FOLDER}${BEHAVIOUR_FILE}
    LOOPFUNCTION_PATH=${EXP_FOLDER}${LOOPFUNCTION_FILE}
	
	for i in `seq ${1} ${2}`; #Creating experiments folder
	do

    	NAME_VARIABLE=${MODEL}_

	    EXP_FILE=${EXP_DES}/${EXP_NAME}_${NAME_VARIABLE}${i}.argos # full path to the experiment configuration file

        EXP_FILE_BIS=${EXP_DES}/${EXP_NAME}_${NAME_VARIABLE}                      	
	    
	    DATA_FILE=${DATA_DES}/${EXP_NAME}_${n}options_${NAME_VARIABLE}${i}.txt # Full path to the data file
	
	    sed -e "s|exp_length|${EXP_LENGTH}|"           \
		-e "s|randomseed|$(($i*124))|"                 \
		-e "s|behaviourpath|${BEHAVIOUR_PATH}|"        \
        -e "s|loopfunctionpath|${LOOPFUNCTION_PATH}|"  \
		-e "s|data_file|${DATA_FILE}|"                 \
		-e "s|datafrequency|${DATA_FREQUENCY}|"        \
		-e "s|num_robots|${NUM_ROBOTS}|"               \
		-e "s|quorumvalue|${QUORUM}|"                  \
        -e "s|commrng|${COMMRANGE}|"                   \
        -e "s|numberofgpscells|${GPS_CELLS_NO}|"       \
        -e "s|robpop1|${ROBPOP1}|"                     \
        -e "s|robpop2|${ROBPOP2}|"                     \
        -e "s|robpop3|${ROBPOP3}|"                     \
        -e "s|robpop4|${ROBPOP4}|"                     \
        -e "s|robpop5|${ROBPOP5}|"                     \
		        ${EXP_TEMPLATE_SRC} > ${EXP_FILE}

        # TODO: when we know ho to schedule on cluster move this outside the loop
        COMMAND="argos3 -c ${EXP_FILE}"
        ${COMMAND}
        echo experiment ${i} finished...
    done
fi
