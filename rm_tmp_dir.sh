#!/bin/sh

for FILE in ${HOME}/Programs/adaptive_symmetry_breaking/job_cluster/*;
do
  # from here
    JOBNAME="$(basename "${FILE}" | sed 's/\(.*\)\..*/\1/')"
    USERNAME="taust"
    TMPDIR=/tmp/$USERNAME/${JOBNAME}
    JOBDIR=/home/$USERNAME/Programs/adaptive_symmetry_breaking/

    rmdir -p $TMPDIR &> /dev/null
done