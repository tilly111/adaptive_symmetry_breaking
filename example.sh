#!/bin/bash
# New Majorana Templates
# Template of a script to execute a python program using mpi4py on the cluster
set -e
set -o pipefail

if [ $# == 0 ]; then
    echo "Usage: ./python-mpi-cluster <EXPERIMENT> <RESULT DIRECTORY>--rack <rack_number> --queue <queue> "
    exit 1
fi

EXPERIMENT=$1 #python file to execute
shift 1
EXECDIR=$2 #where the output produced by the experiment will be saved
shift 1
NB_SLAVES=10 #Number of cores 
RACK_NUM=1
QUEUE_ARG=long
PARAMS=
while [ $# -gt 0 ]; do
    case "$1" in
        --rack) shift; RACK_NUM="$1"; shift;;
        --queue) shift; QUEUE_ARG="$1"; shift;;
        *) PARAMS="$PARAMS $1"; shift;;# terminate case
  esac
done

if [ $NB_SLAVES -lt 2 ]; then
    echo "$0: error: --parallel must be larger than 1"
    exit 1
fi

QUEUE=long
case $QUEUE_ARG in
        long) QUEUE=long;;
        short) QUEUE=short;;
        *) ;;
esac
JOBNAME=ALS_SRUN-$$
MACHINE=7
case $RACK_NUM in
        4)MACHINE=4;;
        5)MACHINE=5;;
        6)MACHINE=Xeon6138;;
        7)MACHINE=Epyc7452;;
        *) ;;
esac
MPIRUN=/opt/ohpc/pub/mpi/openmpi4-gnu9/4.0.5/bin/mpirun
USERNAME=`whoami`
TMPDIR=/tmp/$USERNAME/RESULTS_$JOBNAME

let NB_PARALLEL_PROCESS=NB_SLAVES+1
CURRENT_DIR=`pwd`
exec sbatch <<EOF
#!/bin/sh
#SBATCH -J $JOBNAME
#SBATCH -p $MACHINE # In slurm this is the partition you want to use
#SBATCH -q $QUEUE  # In slurm this is the QoS rule you want to use
##SBATCH -N $NB_PARALLEL_PROCESS  #This specify the number of nodes you want to use, not needed
#SBATCH -n $NB_PARALLEL_PROCESS 
#SBATCH -o $EXECDIR/$JOBNAME-$$.stdout
#SBATCH -e $EXECDIR/$JOBNAME-$$.stderr
##SBATCH -wd $CURRENT_DIR # altough this is the default for slurm
export OMPI_MCA_plm_rsh_disable_qrsh=1
export PATH
export LD_LIBRARY_PATH
mkdir -p ${TMPDIR}
cd $EXECDIR
export PYTHONPATH=${PYTHONPATH}:$PATH_TO_EXPERIMENT

python3.6 -m mpi4py.futures $EXPERIMENT  &>> $TMPDIR/output_$JOBNAME.txt

RET=$?
mv ${TMPDIR}/* $EXECDIR
cd $EXECIDR
rmdir -p ${TMPDIR} &> /dev/null
EOF
