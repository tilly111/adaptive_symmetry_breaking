#!/bin/sh
#SBATCH -J jobname
#SBATCH --ntasks=1                    # Run on a single CPU
#SBATCH --mem=1gb                     # Job memory request
#SBATCH --time=hrs:min:00             # Time limit
#SBATCH --output=/home/taust/log/logfile.txt
#SBATCH --qos=queue                 # this can either be short/special (max 1 day) or long (max 7 days)