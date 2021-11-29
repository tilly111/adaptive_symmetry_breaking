#!/bin/sh                                                                                                                                                    
#SBATCH -J symbreak                                                                                                                                          
#SBATCH --ntasks=1                    # Run on a single CPU                                                                                                  
#SBATCH --mem=1gb                     # Job memory request                                                                                                   
#SBATCH --time=00:05:00               # Time limit hrs:min:sec                                                                                               
#SBATCH -q short                                                                                                                                             

argos3 -c ${1}

#RET=$?                                                                                                                                                      
mv ${2} /home/taust/
#cd $EXECIDR                                                                                                                                                 
#rmdir -p ${TMPDIR} &> /dev/null   