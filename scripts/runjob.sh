#!/bin/sh
## THIS IS ONLY FOR CLUSTER

# setup for the cluster
#SBATCH -J symbreak
#SBATCH --ntasks=1                    # Run on a single CPU
#SBATCH --mem=1gb                     # Job memory request
#SBATCH --time=00:05:00               # Time limit hrs:min:sec
#SBATCH -q short

# check arguments
EXPECTED_ARGS=4
if [ $# -lt ${EXPECTED_ARGS} ]; then
echo "This script generates N experiment files (.argos files)."
echo "Usage $0 <start> <end> <options_size> <model_type> <number_of_options>"
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
    
    #constants
    #Visuaisation flag
    MODEL=${3} # should be local, global or adaptive
    VIZ=${4}
    n=${5}
    CLUSTER=${6}
    NUM_ROBOTS=5 # number of robots
    GPS_CELLS_NO=20 # GPS resolution: 20 per meter aka every grid cell

    QUORUM=-0.9 # Quorum to stop experiment  TODO CHANGE BACK
    COMMRANGE=0.0 # Robots' communication range - not needed ?
        
    EXP_LENGTH=5 #length of the in secs

    DATA_FREQUENCY=1 # frequency of saving the experiment data
    
    # folder paths
    PROJECT_FOLDER=home/taust/Programs/adaptive_symmetry_breaking
    
    BEHAVIOUR_FILE=${PROJECT_FOLDER}/build/behaviours/agent_${MODEL}
    LOOPFUNCTION_FILE=${PROJECT_FOLDER}/build/loop_functions/libkilogrid_${MODEL}
    
    #Path variables
    #path to main directory
    #EXP_FOLDER=${HOME}/Programs/adaptive_symmetry_breaking
    
    # path to template - either with/out visualization
    EXP_TEMPLATE_SRC=ARGoS_simulation/experiment/${n}_op_template.argos
    
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
        ${COMMAND}  # argos3 -c ${1}
        
        mv ${2} /home/taust/  # TODO: adjust path
    done
fi

# call it with sbatch *.sh data file
