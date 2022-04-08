#!/bin/sh

for FILE in ${HOME}/Programs/adaptive_symmetry_breaking/job_cluster/*;
do

    COMMAND="sbatch $FILE &> /dev/null"
    while ! ${COMMAND}
    do
      	sleep 300
    done

    echo "$FILE" scheduled...
    # sbatch $FILE &> /dev/null

    # sleep 1  # sleep for one second probably fix submitting stuff

done

