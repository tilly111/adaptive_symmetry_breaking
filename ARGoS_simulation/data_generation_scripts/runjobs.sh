#!/bin/sh

for FILE in ${HOME}/Programs/adaptive_symmetry_breaking/job_cluster/*;
do

    sbatch $FILE &> /dev/null

    # sleep 1  # sleep for one second probably fix submitting stuff
    
done
