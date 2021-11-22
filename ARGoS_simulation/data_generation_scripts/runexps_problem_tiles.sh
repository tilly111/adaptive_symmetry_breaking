#!/bin/bash

###################################
# Synopsis of the script
###################################
EXPECTED_ARGS=3
if [ $# -lt ${EXPECTED_ARGS} ]; then
echo "This script generates N experiment files (.argos files)."
echo "Usage $0 <start> <end> <options_size> <model_type>"
echo $'\t'"[MENDATORY] <start> is the index of the first experiment"
echo $'\t'"[MENDATORY] <end> is a number of experiment files to generate"
echo $'\t'"[MENDATORY] <model_type> is the type of the model (compare or resample)"
exit

else
    
    HPC="i"  #i-->iceberg s-->sharc

    #Experiment name           
    rtype=tiles_local_inhib
    EXP_NAME=${rtype}  # Do not forget to change the experiment name.
    
    #Visuaisation flag
    VIZ=0

    # Constant experiment variables
    n=3 # number of options/sites
    NUM_ROBOTS=50 # number of robots
    GPS_CELLS_NO=20 # GPS resolution: 20 per meter aka every grid cell

    QUORUM=0.9 # Quorum to stop experiment  TODO CHANGE BACK
    INITIAL_OPINION=1 # initial opinion - should be the crappy one -> atm hard coded bc of controller!
    COMMRANGE=0.1 # Robots' communication range - not needed ?
        
    EXP_LENGTH=2400 #length of the in secs

    DATA_FREQUENCY=1 # frequency of saving the experiment data

    MODEL=${3} # should be "compare" or "inhib"
	BEHAVIOUR_FILE=/build/behaviours/agent_local_${MODEL}_tiles # full path to the compiled robot behaviour

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
	
	#EXPS_FOLDER_NAME=myargosexperiments
	
	EXP_FOLDER=${HOME}/Desktop/ProjectsXCode.nosync/argos3-test
	
    if [[ ${VIZ} -eq 1 ]]; then
        EXP_TEMPLATE_SRC=ARGoS_simulation/experiment/tiles_${n}_op_template_viz.argos
    else
        EXP_TEMPLATE_SRC=ARGoS_simulation/experiment/tiles_${n}_op_template.argos
    fi
    
	EXP_DES=${EXP_FOLDER}/experiments_cluster/${n}options_N${NUM_ROBOTS}_model${MODEL}
	mkdir -p ${EXP_DES}

	DATA_DES=${EXP_FOLDER}/data_cluster/${n}options_N${NUM_ROBOTS}_${EXP_NAME}
	mkdir -p ${DATA_DES}

	BEHAVIOUR_PATH=${EXP_FOLDER}${BEHAVIOUR_FILE}
	
	for i in `seq ${1} ${2}`; #Creating experiments folder
	do

    	NAME_VARIABLE=${NUM_ROBOTS}_

	    EXP_FILE=${EXP_DES}/${EXP_NAME}_${NAME_VARIABLE}${i}.argos # full path to the experiment configuration file

        EXP_FILE_BIS=${EXP_DES}/${EXP_NAME}_${NAME_VARIABLE}                      	
	    
	    DATA_FILE=${DATA_DES}/${EXP_NAME}_${n}options_${NAME_VARIABLE}${i}.txt # Full path to the data file
	
	    sed -e "s|exp_length|${EXP_LENGTH}|"           \
		-e "s|randomseed|$(($i*124))|"                 \
		-e "s|behaviourpath|${BEHAVIOUR_PATH}|"        \
        -e "s|viz|${VIZ}|"        \
		-e "s|expfolder|${EXP_FOLDER}|"                \
		-e "s|data_file|${DATA_FILE}|"                 \
		-e "s|datafrequency|${DATA_FREQUENCY}|"        \
		-e "s|num_robots|${NUM_ROBOTS}|"               \
		-e "s|quorumvalue|${QUORUM}|"                  \
        -e "s|commrng|${COMMRANGE}|"                   \
        -e "s|initial_opinion_var|${INITIAL_OPINION}|" \
		-e "s|numberofgpscells|${GPS_CELLS_NO}|"       \
		        ${EXP_TEMPLATE_SRC} > ${EXP_FILE}

        COMMAND="argos3 -c ${EXP_FILE}"
        ${COMMAND}
        echo experiment ${i} finished...
    done
fi
