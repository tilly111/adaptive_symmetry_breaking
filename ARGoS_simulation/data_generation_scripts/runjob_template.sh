#!/bin/sh
#SBATCH -J jobname
#SBATCH --ntasks=1                    # Run on a single CPU
#SBATCH --mem=1gb                     # Job memory request
#SBATCH --time=hrs:min:00             # Time limit
#SBATCH -q short
#SBATCH --output=/home/taust/log/logfile.txt

argos3 -c argosfile > /home/taust/log/

mv savefile wheresave
