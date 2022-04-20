#!/bin/sh

for FILE in ${HOME}/Programs/adaptive_symmetry_breaking/job_cluster/*;
do
  # from here
    JOBNAME="$(basename "${FILE}" | sed 's/\(.*\)\..*/\1/')"
    USERNAME="taust"
    TMPDIR=/tmp/$USERNAME/jobname
    JOBDIR=/home/$USERNAME/Programs/adaptive_symmetry_breaking/
    mkdir -p $TMPDIR
    mv * $TMPDIR
    cd $TMPDIR
    # till here

    COMMAND="sbatch $FILE &> /dev/null"
    while ! ${COMMAND}
    do
      	sleep 300
    done

    echo "$FILE" scheduled...
# from here
  RET=$?
  mv * $JOBDIR
  cd $JOBDIR
  rmdir -p $TMPDIR &> /dev/null
  exit $RET
  # till here
done
