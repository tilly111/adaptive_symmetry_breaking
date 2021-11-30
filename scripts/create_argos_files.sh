#!/bin/bash

###################################
# Synopsis of the script
###################################
EXPECTED_ARGS=4
if [ $# -lt ${EXPECTED_ARGS} ]; then
echo "This script generates N experiment files (.argos files)."
echo "Usage $0 <start> <end> <options_size> <model_type> <visualization> <number_of_options> <on_cluster>"
echo $'\t'"[MANDATORY] <start> is the index of the first experiment"
echo $'\t'"[MANDATORY] <end> is the index of the last experiment"
echo $'\t'"[MANDATORY] <model_type> is the type of the model (local, global or adaptive)"
echo $'\t'"[MANDATORY] <number_of_options> the number of possible options (3 or 5)"
exit

else
    #Varibales to change
    # Experiment name
    rtype=cluster_test
    EXP_NAME=${rtype}  # Do not forget to change the experiment name.
    
    #TODO: atm hard coded but make it dynamic -> uncommited state to be implemented maybe 0?
    ROBPOP1=5
    ROBPOP2=0
    ROBPOP3=0
    ROBPOP4=0
    ROBPOP5=0
    NUM_ROBOTS=5 # number of robots
    
    #constants
    #Visuaisation flag
    MODEL=${3} # should be local, global or adaptive
    n=${4}
    GPS_CELLS_NO=20 # GPS resolution: 20 per meter aka every grid cell

    QUORUM=-0.9 # Quorum to stop experiment  TODO CHANGE BACK
    COMMRANGE=0.0 # Robots' communication range - not needed ?
        
    EXP_LENGTH=5 #length of the in secs

    DATA_FREQUENCY=1 # frequency of saving the experiment data
    
    HRS=01  # hours the script takes
    MIN=00  # min the script takes
    
    #path to main directory
    EXP_FOLDER=${HOME}/Programs/adaptive_symmetry_breaking
    
    # full path to the compiled robot behaviour/loopfunction
    BEHAVIOUR_FILE=${EXP_FOLDER}/build/behaviours/agent_${MODEL}
    LOOPFUNCTION_FILE=${EXP_FOLDER}/build/loop_functions/libkilogrid_${MODEL}
    
    
    # path to template
    EXP_TEMPLATE_SRC=${EXP_FOLDER}/ARGoS_simulation/experiment/${n}_op_template.argos
    JOB_TEMPLATE_SRC=${EXP_FOLDER}/scripts/runjob_template.sh
    
    EXP_DES=${EXP_FOLDER}/experiments_cluster/${n}options_N${NUM_ROBOTS}_model${MODEL}
    mkdir -p ${EXP_DES}

    DATA_DES=${EXP_FOLDER}/data_cluster/${n}options_N${NUM_ROBOTS}_${MODEL}_${EXP_NAME}
    mkdir -p ${DATA_DES}
    
    JOB_DIR=${EXP_FOLDER}/job_cluster
    mkdir -p ${JOB_DIR}

    for i in `seq ${1} ${2}`; #Creating experiments folder
    do

        EXP_FILE=${EXP_DES}/${EXP_NAME}_${MODEL}${i}.argos # full path to the experiment configuration file
        JOB_FILE=${JOB_DIR}/${EXP_NAME}_${NUM_ROBOTS}_${n}_model_${MODEL}_${i}.sh
        
        DATA_FILE=${EXP_NAME}_${n}options_${MODEL}${i}.txt # Full path to the data file
        
    
        sed -e "s|exp_length|${EXP_LENGTH}|"           \
        -e "s|randomseed|$(($i*124))|"                 \
        -e "s|behaviourpath|${BEHAVIOUR_FILE}|"        \
        -e "s|loopfunctionpath|${LOOPFUNCTION_FILE}|"  \
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
        
        sed -e "s|jobname|${JOB_NAME}|" \
        -e "s|min|${MIN}|"              \
        -e "s|hrs|${HRS}|"              \
        -e "s|argosfile|${EXP_FILE}|"   \
        -e "s|savefile|${DATA_FILE}|"   \
        -e "s|wheresave|${DATA_DES}/|"  \
                ${JOB_TEMPLATE_SRC} > ${JOB_FILE}
    done
fi

#JOB_NAME=${rtype}_${NUM_ROBOTS}_${OPTIONSIZE}_model${MODEL}
#    sed -e "s|jobname|${JOB_NAME}|"   \
#        -e "s|min|${MIN}|"   \
#        -e "s|hrs|${HRS}|"   \
#        -e "s|argosfile|${execfile}" \  # full path to the .argos file to execute
#        -e "s|savefile|${savefile}" \   # the file we save to
#        runjob_template.sh > runjob.sh
#
