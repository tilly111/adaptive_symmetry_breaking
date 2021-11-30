#!/bin/sh
#SBATCH -J jobname
#SBATCH --ntasks=1                    # Run on a single CPU
#SBATCH --mem=1gb                     # Job memory request
#SBATCH --time=hrs:min:00             # Time limit
#SBATCH -q short
argos3 -c argosfile

mv ${2} /home/taust/experiments
                                                

JOB_NAME=${rtype}_${NUM_ROBOTS}_${OPTIONSIZE}_model${MODEL}
    sed -e "s|jobname|${JOB_NAME}|"   \
        -e "s|min|${MIN}|"   \
        -e "s|hrs|${HRS}|"   \
        -e "s|argosfile|${execfile}" \  # full path to the .argos file to execute
        -e "s|savefile|${savefile}" \   # the file we save to
        runjob_template.sh > runjob.sh
