#!/bin/bash
tmp_counter=0
com_range_counter=1
for j in `seq 63 81`;
    do
      conf=ASB_experiment_$((1 + ${tmp_counter} % 3)).kconf
      n=$((3 + ${tmp_counter} % 3))

      if ((${tmp_counter} == 3)); then
        com_range_counter=2
      elif ((${tmp_counter} == 6)); then
        com_range_counter=5
      elif ((${tmp_counter} == 9)); then
        com_range_counter=10
      elif ((${tmp_counter} == 12)); then
        com_range_counter=20
      elif ((${tmp_counter} == 15)); then
        com_range_counter=45
      fi
      INITIAL_COMMUNICATION_RANGE=${com_range_counter}
      EXP_NAME=experiment_adaptation_${j}_comrng_${INITIAL_COMMUNICATION_RANGE}

      tmp_counter=$(( ${tmp_counter} + 1 ))
      echo ${EXP_NAME} ${conf} ${n}
    done