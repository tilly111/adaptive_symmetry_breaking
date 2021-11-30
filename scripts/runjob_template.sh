#!/bin/sh
#SBATCH -J jobname
#SBATCH --ntasks=1                    # Run on a single CPU
#SBATCH --mem=1gb                     # Job memory request
#SBATCH --time=hrs:min:00             # Time limit
#SBATCH -q short
argos3 -c argosfile

mv savefile wheresave
